import collections
import collections.abc

# 1. ПАТЧ СУМІСНОСТІ (має бути ПЕРШИМ)
collections.MutableMapping = collections.abc.MutableMapping

import time
import math
import sys
from dronekit import connect, VehicleMode, APIException, LocationGlobalRelative

# --- КОНФІГУРАЦІЯ ---
CONNECTION_STRING = 'tcp:127.0.0.1:5762'

# --- ЗАВДАННЯ ---
TARGET_LOCATION = LocationGlobalRelative(50.443326, 30.448078, 300) 

WIND_PARAMS = {
    'SIM_WIND_SPD': 5,
    'SIM_WIND_DIR': 30,
    'SIM_WIND_TURB': 2,
    'SIM_WIND_T_FREQ': 0.2 # Виправлена назва параметру SITL
}

# Коефіцієнти PID (налаштовані під агресивне утримання і боротьбу з вітром)
Gains = {
    'Alt':   {'Kp': 6.0, 'Ki': 0.6, 'Kd': 10.0, 'iMax': 400, 'iZone': 30.0}, 
    'Pos':   {'Kp': 35.0, 'Ki': 0.8, 'Kd': 15.0, 'iMax': 400, 'iZone': 50.0},
    'Nav':   {'Kp': 35.0, 'Ki': 0.8, 'Kd': 15.0, 'iMax': 400, 'iZone': 50.0},
    'Yaw':   {'Kp': 3.0, 'Ki': 0.01, 'Kd': 0.5, 'iMax': 50, 'iZone': 20.0},
}

# --- КЛАС: PID КОНТРОЛЕР ---
class PIDController:
    def __init__(self, Kp, Ki, Kd, iMax=None, iZone=None):
        self.Kp, self.Ki, self.Kd, self.iMax = Kp, Ki, Kd, iMax
        
        # Налаштування зони інтеграції
        if iZone is not None:
            self.iZone = iZone
        elif self.Kp > 0:
            self.iZone = 20.0 
        else:
            self.iZone = 0.0
            
        self.reset()

    def reset(self):
        self._last_error = None
        self._integrator = 0.0
        self._last_time = time.time()

    def update(self, error):
        dt = time.time() - self._last_time
        if dt <= 0.0: dt = 1e-6
        
        # Зонована інтеграція (I-Zone)
        if self.iZone == 0.0 or abs(error) <= self.iZone:
            self._integrator += error * self.Ki * dt
            if self.iMax: self._integrator = max(-self.iMax, min(self.iMax, self._integrator))
        else:
            self._integrator *= 0.95 # Плавне затухання замість жорсткого скидання до 0
            
        if self._last_error is None:
            self._last_error = error
            
        output = error * self.Kp + self._integrator + (error - self._last_error) * self.Kd / dt
        self._last_error, self._last_time = error, time.time()
        return output

# --- НАВІГАЦІЙНА МАТЕМАТИКА (ВИПРАВЛЕНА ГЕОМЕТРІЯ ДЛЯ 50-ї ШИРОТИ) ---
def get_location_distance_meters(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    # Компенсація довготи через косинус широти
    dlon = (loc2.lon - loc1.lon) * math.cos(math.radians((loc1.lat + loc2.lat) / 2.0))
    return math.sqrt((dlat**2) + (dlon**2)) * 1.113195e5

def get_location_bearing(loc1, loc2):
    off_y = loc2.lat - loc1.lat
    # Компенсація довготи через косинус широти
    off_x = (loc2.lon - loc1.lon) * math.cos(math.radians((loc1.lat + loc2.lat) / 2.0))
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0: bearing += 360.00
    return bearing

# --- ГОЛОВНА МІСІЯ ---
def execute_flight_mission(vehicle, target_loc, log_file):
    print("\n--- СТАРТ ІНТЕЛЕКТУАЛЬНОЇ ПОЛЬОТНОЇ МІСІЇ ---")
    
    print("Готуємось до Arming...")
    vehicle.mode = VehicleMode("STABILIZE")
    vehicle.channels.overrides = {'3': 1000}
    while not vehicle.is_armable: time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed: time.sleep(1)
    print("[OK] Двигуни активні.")

    home_loc = vehicle.location.global_relative_frame
    target_altitude = target_loc.alt
    initial_heading = vehicle.heading # Запам'ятовуємо стабільний Yaw
    
    def write_telemetry(dist_tgt, xte, alt_err):
        loc = vehicle.location.global_relative_frame
        ovr = vehicle.channels.overrides
        log_file.write(
            f"{time.time():.2f},{loc.lat:.6f},{loc.lon:.6f},{loc.alt:.2f},"
            f"{dist_tgt:.2f},{xte:.2f},{alt_err:.2f},"
            f"{ovr.get('1', 1500)},{ovr.get('2', 1500)},{ovr.get('3', 1500)},{ovr.get('4', 1500)},"
            f"{vehicle.heading},{math.degrees(vehicle.attitude.roll):.2f},{math.degrees(vehicle.attitude.pitch):.2f},"
            f"{vehicle.groundspeed:.2f}\n"
        )

    total_distance = get_location_distance_meters(home_loc, target_loc)
    ideal_bearing = get_location_bearing(home_loc, target_loc)
    print(f"[NAV] Дистанція: {total_distance:.1f} м, Курс: {ideal_bearing:.1f}°")

    alt_pid = PIDController(**Gains['Alt'])
    pos_pid = PIDController(**Gains['Pos'])
    nav_pid = PIDController(**Gains['Nav'])
    yaw_pid = PIDController(**Gains['Yaw'])

    print("Вертикальний зліт...")
    takeoff_target_alt = 5.0
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        alt_error = takeoff_target_alt - current_alt
        thr = 1515 + alt_pid.update(alt_error)
        
        # Утримання Yaw від вітру прямо на зльоті
        yaw_err = initial_heading - vehicle.heading
        if yaw_err > 180: yaw_err -= 360
        if yaw_err < -180: yaw_err += 360
        yaw_out = 1500 + yaw_pid.update(yaw_err)
        
        if current_alt > 4.5: 
            break
            
        vehicle.channels.overrides = {
            '1': 1500, '2': 1500, 
            '3': int(max(1100, min(1800, thr))), 
            '4': int(max(1100, min(1900, yaw_out)))
        }
        write_telemetry(total_distance, 0.0, alt_error)
        print(f"  Висота: {current_alt:.1f}м | Yaw: {vehicle.heading:.1f}°", end='\r')
        time.sleep(0.1)
        
    print("\n[OK] Дрон на стартовій висоті.")

    # Плавне нарощування цільової висоти щоб не було стрибка газів
    current_flight_alt = takeoff_target_alt
    
    # Скидаємо накопичення після фази зльоту
    alt_pid.reset()
    pos_pid.reset()
    nav_pid.reset()
    yaw_pid.reset()

    try:
        flight_start_time = time.time()
        while time.time() - flight_start_time < 600:
            current_loc = vehicle.location.global_relative_frame
            dist_to_target = get_location_distance_meters(current_loc, target_loc)
            
            dist_from_home = get_location_distance_meters(home_loc, current_loc)
            bearing_hc = get_location_bearing(home_loc, current_loc)
            
            angle_diff = bearing_hc - ideal_bearing
            if angle_diff > 180: angle_diff -= 360
            if angle_diff < -180: angle_diff += 360
            angle_diff_rad = math.radians(angle_diff)

            # Проекція на ідеальну лінію
            dist_along = dist_from_home * math.cos(angle_diff_rad)
            err_along = total_distance - dist_along
            xte_error = dist_from_home * math.sin(angle_diff_rad)

            out_fwd = pos_pid.update(err_along)
            out_side = nav_pid.update(-xte_error)

            # --- ПРЕЕМПТИВНЕ ГАЛЬМУВАННЯ ---
            # Розрахунок безпечної швидкості (з уповільненням 1.5 м/с^2) для поточного відрізка
            if dist_to_target < 200.0:
                v_ideal = math.sqrt(2 * 1.5 * max(0.1, dist_to_target))
                if vehicle.groundspeed > v_ideal:
                    # Агресивне гальмування: множник 200.0 змусить його різко задерти ніс!
                    brake_force = (vehicle.groundspeed - v_ideal) * 200.0
                    out_fwd -= brake_force

            # --- ГЕЛІКОПТЕРНИЙ ПРОФІЛЬ ---
            # Визначаємо, скільки ще потрібно набрати до цільової висоти
            true_alt_error = target_altitude - current_loc.alt
            
            if true_alt_error > 50.0:
                # Якщо ми ще не набрали 250м -> обмежуємо розгін до 100 PWM
                max_pitch_roll = 100.0
            elif true_alt_error < 10.0:
                # Якщо до ешелону залишилось менше 10м -> даємо повну тягу (400 PWM)
                max_pitch_roll = 400.0
            else:
                # Плавна інтерполяція від 100 до 400 PWM між висотами 250м і 290м
                proportion = (50.0 - true_alt_error) / 40.0
                max_pitch_roll = 100.0 + (proportion * 300.0)

            out_fwd = max(-max_pitch_roll, min(max_pitch_roll, out_fwd)) 
            out_side = max(-max_pitch_roll, min(max_pitch_roll, out_side))

            # Трансформація координат
            diff_rad = math.radians(ideal_bearing - vehicle.heading)
            pitch_offset = out_fwd * math.cos(diff_rad) - out_side * math.sin(diff_rad)
            roll_offset = out_fwd * math.sin(diff_rad) + out_side * math.cos(diff_rad)

            pitch_pwm = max(1150, min(1850, 1500 - pitch_offset))
            roll_pwm = max(1150, min(1850, 1500 + roll_offset))

            if dist_to_target < 4.0:
                print("\n[FINISH] Ціль досягнута. Перехід до посадки...")
                break

            # Плавна зміна цільової висоти
            if current_flight_alt < target_altitude:
                current_flight_alt += 2.0 # плавний підйом
            
            alt_error = current_flight_alt - current_loc.alt
            thr_pwm = 1500 + alt_pid.update(alt_error)

            # Активне утримання ідеального Yaw
            yaw_err = initial_heading - vehicle.heading
            if yaw_err > 180: yaw_err -= 360
            if yaw_err < -180: yaw_err += 360
            yaw_out = 1500 + yaw_pid.update(yaw_err)
            
            overrides = {
                '1': int(roll_pwm),
                '2': int(pitch_pwm),
                '3': int(max(1100, min(1800, thr_pwm))),
                '4': int(max(1100, min(1900, yaw_out)))
            }
            vehicle.channels.overrides = overrides
            
            write_telemetry(dist_to_target, xte_error, alt_error)
            print(f"D:{dist_to_target:.1f}m XTE:{xte_error:.1f}m Alt:{current_loc.alt:.1f}m | T:{overrides['3']} Y:{overrides['4']}", end='\r')
            time.sleep(0.1)

        print("\nПочинаємо фінальну посадку у точці Б...")
        landing_target_alt = current_loc.alt
        
        # Базове скидання перед початком посадкового маневру
        alt_pid.reset() 
        yaw_pid.reset()
        pos_pid.reset()
        nav_pid.reset()
        
        # Перехід у режим точного зависання: піднімаємо I-term, 
        # щоб миттєво долати похибку від зсуву вітру (Wind Shear) при спуску
        pos_pid.Ki = 3.0
        nav_pid.Ki = 3.0
        
        # Прапорці фазового контролю (Адаптація до висотних шарів вітру)
        wind_zone_150 = False
        wind_zone_50 = False
        wind_zone_10 = False
        
        while vehicle.location.global_relative_frame.alt > 0.5:
            curr_loc = vehicle.location.global_relative_frame
            
            # Каскадне скидання інтегратора під стихаючий приземний вітер
            if curr_loc.alt < 150.0 and not wind_zone_150:
                pos_pid.reset()
                nav_pid.reset()
                wind_zone_150 = True
                print("\n[WIND SHEAR] Перетин ешелону 150м - Адаптація ПІД...")
                
            if curr_loc.alt < 50.0 and not wind_zone_50:
                pos_pid.reset()
                nav_pid.reset()
                wind_zone_50 = True
                print("\n[WIND SHEAR] Перетин ешелону 50м - Адаптація ПІД...")
                
            if curr_loc.alt < 10.0 and not wind_zone_10:
                pos_pid.reset()
                nav_pid.reset()
                wind_zone_10 = True
                print("\n[WIND SHEAR] Перетин ешелону 10м (Фінальна посадка) - Адаптація ПІД...")
            
            # Поступовий спуск 2.5 м/с 
            landing_target_alt = max(0.0, landing_target_alt - 0.25)
            
            a_err = landing_target_alt - curr_loc.alt
            thr_pwm = 1495 + alt_pid.update(a_err)
            
            dist_from_home = get_location_distance_meters(home_loc, curr_loc)
            bearing_hc = get_location_bearing(home_loc, curr_loc)
            
            angle_diff = bearing_hc - ideal_bearing
            if angle_diff > 180: angle_diff -= 360
            if angle_diff < -180: angle_diff += 360
            angle_diff_rad = math.radians(angle_diff)

            dist_along = dist_from_home * math.cos(angle_diff_rad)
            err_along = total_distance - dist_along
            xte_err = dist_from_home * math.sin(angle_diff_rad)

            out_fwd = pos_pid.update(err_along)
            out_fwd = max(-400, min(400, out_fwd))
            
            out_side = nav_pid.update(-xte_err)
            out_side = max(-400, min(400, out_side))

            diff_rad = math.radians(ideal_bearing - vehicle.heading)
            pitch_offset = out_fwd * math.cos(diff_rad) - out_side * math.sin(diff_rad)
            roll_offset = out_fwd * math.sin(diff_rad) + out_side * math.cos(diff_rad)

            pitch_pwm = max(1150, min(1850, 1500 - pitch_offset))
            roll_pwm = max(1150, min(1850, 1500 + roll_offset))

            yaw_err = initial_heading - vehicle.heading
            if yaw_err > 180: yaw_err -= 360
            if yaw_err < -180: yaw_err += 360
            yaw_out = 1500 + yaw_pid.update(yaw_err)
            
            vehicle.channels.overrides = {
                '1': int(roll_pwm),
                '2': int(pitch_pwm),
                '3': int(max(1000, min(1800, thr_pwm))),
                '4': int(max(1100, min(1900, yaw_out)))
            }
            
            dist_tgt = get_location_distance_meters(curr_loc, target_loc)
            write_telemetry(dist_tgt, xte_err, a_err)
            print(f"  Посадка... Висота: {curr_loc.alt:.1f}м | Відхилення: {dist_tgt:.1f}м", end='\r')
            time.sleep(0.1)
            
        print("\n[SUCCESS] Дрон на землі. Вимикаємо двигуни...")
        vehicle.channels.overrides = {'1': 1500, '2': 1500, '3': 1000, '4': 1500}
        time.sleep(2)
        
        vehicle.armed = False
        while vehicle.armed:
            print("  Чекаємо на зупинку гвинтів...", end='\r')
            time.sleep(1)
            
        print("\n[FINISH] Двигуни зупинено. Місія виконана на 100%!")

    except Exception as e:
        error_message = f"\n[FATAL] Помилка в циклі керування: {e}"
        print(error_message, file=sys.stderr)
        if log_file: log_file.write(f"FATAL ERROR in mission loop: {e}\n")

# --- ГОЛОВНА ФУНКЦІЯ ---
def main():
    print(f"--- Підключення до ArduPilot SITL: {CONNECTION_STRING} ---")
    vehicle, log_file = None, None
    try:
        log_file = open('app/mission_log.txt', 'w', encoding='utf-8')
        log_file.write("Timestamp,Lat,Lon,Alt,DistToTarget,XTE,AltError,RollPWM,PitchPWM,ThrottlePWM,YawPWM,Heading,RollDeg,PitchDeg,GroundSpeed\n")
        
        vehicle = connect(CONNECTION_STRING, wait_ready=True, timeout=60)
        print("[SUCCESS] Зв'язок встановлено.")
        
        print("\n--- Оновлення умов симуляції ---")
        for key, val in WIND_PARAMS.items(): vehicle.parameters[key] = val
        print(" [OK] Параметри вітру прийнято.")
        
        execute_flight_mission(vehicle, TARGET_LOCATION, log_file)

    except APIException as e:
        error_message = f"\n[API ERROR] Помилка: {e}"
        print(error_message, file=sys.stderr)
        if log_file: log_file.write(f"API CONNECTION ERROR: {e}\n")
    except Exception as e:
        error_message = f"\n[ERROR] Сталася помилка: {e}"
        print(error_message, file=sys.stderr)
        if log_file: log_file.write(f"GENERAL SCRIPT ERROR: {e}\n")
    finally:
        if vehicle:
            print("\n--- Безпечне завершення ---")
            vehicle.channels.overrides = {}
            vehicle.close()
            print("З'єднання закрито.")
        if log_file:
            log_file.close()
            print("Файл логів закрито.")

if __name__ == "__main__":
    main()
