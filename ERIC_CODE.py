#!/usr/bin/env python3
from ev3dev2.motor import MediumMotor, OUTPUT_D, OUTPUT_C, OUTPUT_A
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
import time
import threading

# Настройки поиска и объезда
TARGET_VALUE = 30
BLACK_THRESHOLD = 10 

# Настройки езды по линии
VALUE = 20
BLACK = 5
LINE_SPEED = 30
command = True
Kp = 1.5
Kd = 2
last_error = 0

# Настройки поворотов на 90 градусов
MAX_U = 100          # ограничение вращения
HARD_U = 80          # вращение в жёстком повороте (было 100, меньше заброс)
HARD_BASE = -40      # движение назад в жёстком повороте (0 = чистый разворот на месте)
MIN_SPEED = 8        # минимальная скорость вперёд при сильной коррекции
SLOW_K = 0.4         # насколько сильно сбрасываем скорость вперёд при коррекции

# Моторы и сенсоры
MOTOR_L = MediumMotor(OUTPUT_C)
MOTOR_R = MediumMotor(OUTPUT_D)
MOTOR_B = MediumMotor(OUTPUT_A)

COLOR_SENSOR = ColorSensor(INPUT_2)
COLOR_SENSOR.mode = 'COL-REFLECT'

DIST_LEFT = UltrasonicSensor(INPUT_1)
DIST_RIGHT = UltrasonicSensor(INPUT_3)
DIST_FRONT = UltrasonicSensor(INPUT_4)

BASE_SPEED = 100

def move_omni(speed_l, speed_r, speed_b):
    MOTOR_L.on(speed_l)
    MOTOR_R.on(speed_r)
    MOTOR_B.on(speed_b)

def clamp(x, lo, hi):
    return max(min(x, hi), lo)

def drive_line(base, u):
    # base: движение вперёд, u: вращение вокруг центра
    speed_b = -u
    speed_r = base - u
    speed_l = -base - u

    # Если какой-то мотор выходит за 100, масштабируем все три вместе,
    # чтобы сохранить направление движения (раздельная обрезка
    # смещала центр вращения и платформу "уводило" в повороте)
    biggest = max(abs(speed_b), abs(speed_r), abs(speed_l))
    if biggest > 100:
        k = 100.0 / biggest
        speed_b *= k
        speed_r *= k
        speed_l *= k

    MOTOR_B.on(speed_b)
    MOTOR_R.on(speed_r)
    MOTOR_L.on(speed_l)

def turn_180():
    move_omni(BASE_SPEED, -BASE_SPEED, 0)
    time.sleep(0.5) 
    move_omni(-BASE_SPEED, -BASE_SPEED, -BASE_SPEED)
    time.sleep(0.8) 

def evade_obstacle(direction):
    if direction == 'front':
        move_omni(BASE_SPEED, BASE_SPEED, BASE_SPEED)
    elif direction == 'left':
        move_omni(-BASE_SPEED, -BASE_SPEED, -BASE_SPEED)
    elif direction == 'right':
        move_omni(BASE_SPEED, BASE_SPEED, BASE_SPEED)
    #if direction == 'front':
        #move_omni(BASE_SPEED, BASE_SPEED, BASE_SPEED)
    #elif direction == 'left':
        #move_omni(-BASE_SPEED // 1.5, -BASE_SPEED // 1.5, BASE_SPEED)
    #elif direction == 'right':
        #move_omni(BASE_SPEED // 1.5, BASE_SPEED // 1.5, -BASE_SPEED)

# --- Фоновая функция для чтения терминала ---
def command_listener():
    global command
    while True:
        user_input = input().strip().lower()
        if user_input in ['true', 't', '1']:
            command = True
            print("Line mode")
        elif user_input in ['false', 'f', '0']:
            command = False
            print("Eblan mode")

# Запуск чтения терминала в параллельном потоке
listener_thread = threading.Thread(target=command_listener, daemon=True)
listener_thread.start()

try:
    # === ПЕРВЫЙ ЦИКЛ: Поиск линии и объезд препятствий ===
    while True:
        light_val = COLOR_SENSOR.reflected_light_intensity
        dist_l = DIST_LEFT.distance_centimeters
        dist_f = DIST_FRONT.distance_centimeters
        dist_r = DIST_RIGHT.distance_centimeters
        
        if light_val < BLACK_THRESHOLD:
            if command == False:
                turn_180()
            else:
                move_omni(0, 0, 0)
                time.sleep(1)
                break # Выходим из первого while True и идем дальше по коду
        
        elif dist_f < TARGET_VALUE:
            evade_obstacle('front')
                    
        elif dist_l < TARGET_VALUE:
            evade_obstacle('left')
                    
        elif dist_r < TARGET_VALUE:
            evade_obstacle('right')   
        else:
            move_omni(-BASE_SPEED, BASE_SPEED, 0)
            
        time.sleep(0.001)

    print("Start the line code")
    intensity = COLOR_SENSOR.reflected_light_intensity
    while intensity >= BLACK:
        intensity = COLOR_SENSOR.reflected_light_intensity
        move_omni(LINE_SPEED, -LINE_SPEED, 0)

    is_hard_turning = False
    while True:
        intensity = COLOR_SENSOR.reflected_light_intensity
        error = VALUE - intensity

        if intensity < BLACK:
            is_hard_turning = True
        
        if is_hard_turning and intensity >= VALUE:
            is_hard_turning = False

        if is_hard_turning:
            current_base_speed = HARD_BASE
            U = HARD_U
        else:
            derivative = error - last_error
            U = Kp * error + Kd * derivative
            U = clamp(U, -MAX_U, MAX_U)
            # Чем сильнее коррекция, тем медленнее едем вперёд:
            # на углу платформа почти разворачивается на месте
            current_base_speed = max(LINE_SPEED - SLOW_K * abs(U), MIN_SPEED)

        # last_error обновляется всегда, чтобы при выходе из жёсткого
        # поворота D-составляющая не получала скачок от нуля
        last_error = error

        drive_line(current_base_speed, U)
        
except KeyboardInterrupt:
    MOTOR_B.off()
    MOTOR_R.off()
    MOTOR_L.off()