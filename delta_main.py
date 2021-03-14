# notify
print('LOAD: delta.py')

import sys, time, math
from machine import Pin

#Piny

#Enable
EN_1 = Pin(2, Pin.OUT)
EN_2 = Pin(4, Pin.OUT)
EN_3 = Pin(5, Pin.OUT)

#Direction
DIR_1 = Pin(12, Pin.OUT)
DIR_2 = Pin(14, Pin.OUT)
DIR_3 = Pin(16, Pin.OUT)

#Step
STP_1 = Pin(12, Pin.OUT)
STP_2 = Pin(14, Pin.OUT)
STP_3 = Pin(19, Pin.OUT)

#Stale konstrukcyjne robota
e = 24
f = 75
re = 300
rf = 100
btf = 400

#Stale modelowe
total_steps = 32000                #Liczba krokow na obrot
delay_durr = 100                  #Odstep pomiedzy krokami [ms]
on_off_durr = 5                     #Czas wlaczenia sygnalu step [ms]

#Stale matematyczne
sqrt3 = math.sqrt(3)
pi = math.pi
sin120 = sqrt3 / 2
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1 / sqrt3

#Globalne parametry polozenia
pos_x = 0
pos_y = 0
pos_z = -200

theta_1 = -78.491
theta_2 = -78.491
theta_3 = -78.491

def update_model(steps, dur, en_dur):
    total_steps = steps
    delay_durr = dur
    on_off_durr = en_dur

def update_const(new_e, new_f, new_re, new_rf, new_btf):
    e = new_e
    f = new_f
    re = new_re
    rf = new_rf
    btf = new_btf

def move_motors(steps1, steps2, steps3):

    if (steps1 < 0):
        d1 = 0
    if (steps1 >= 0):
        d1 = 1
    set_engine_direction(1, d1)
    steps1 = abs(steps1)

    if (steps2 < 0):
        d2 = 0
    if (steps2 >= 0):
        d2 = 1
    set_engine_direction(2, d2)
    steps2 = abs(steps2)

    if (steps3 < 0):
        d3 = 0
    if (steps3 >= 0):
        d3 = 1
    set_engine_direction(3, d3)
    steps3 = abs(steps3)

    enable_engines(1)

    while((steps1 != 0) and (steps2 != 0) and (steps3 != 0)):

        if(steps1 != 0):
            STP_1.value(1)
            time.sleep_ms(on_off_durr)
            STP_1.value(0)
            steps1 -= 1

        if (steps2 != 0):
            STP_2.value(1)
            time.sleep_ms(on_off_durr)
            STP_2.value(0)
            steps2 -= 1

        if (steps3 != 0):
            STP_3.value(1)
            time.sleep_ms(on_off_durr)
            STP_3.value(0)
            steps3 -= 1

        time.sleep_ms(delay_durr)


    enable_engines(0)

def set_engine_direction(eng_num, direction):
    if (eng_num == 1):
        if(direction == 1):
            DIR_1.value(1)
        else:
            DIR_1.value(0)

    if (eng_num == 2):
        if(direction == 1):
            DIR_2.value(1)
        else:
            DIR_2.value(0)

    if (eng_num == 3):
        if(direction == 1):
            DIR_3.value(1)
        else:
            DIR_3.value(0)

def enable_engines(self):
    if (self == 1):
        EN_1.value(1)
        EN_2.value(1)
        EN_3.value(1)
    if (self == 0):
        EN_1.value(0)
        EN_2.value(0)
        EN_3.value(0)

def get_steps(theta1, theta2, theta3):
    steps = []
    steps.append(calc_steps(theta_1, theta1))
    steps.append(calc_steps(theta_2, theta2))
    steps.append(calc_steps(theta_3, theta3))

    return steps

def calc_steps(theta_curr, theta_next):
    angle = theta_curr - theta_next
    el_angle = 360 / total_steps
    step = angle / el_angle

    return round(step)

def inverse_kinematics(x, y, z):
    angles = []
    angles.append(calc_angle(x, y, z))
    angles.append(calc_angle(x * cos120 + y * sin120, y * cos120 - x * sin120, z))
    angles.append(calc_angle(x * cos120 - y * sin120, y * cos120 + x * sin120, z))

    if ((angles[0] == 0) or (angles[1] == 0) or (angles[2] == 0)):
        angles.append(-1)
    else:
        angles.append(0)

    return angles

def calc_angle(x0, y0, z0):
    y1 = -0.5 * 0.57735 * f
    y0 -= 0.5 * 0.57735 * e

    a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0)
    b = (y1 - y0) / z0

    d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
    if(d < 0):
        return 0

    yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1)
    zj = a + b * yj
    if (yj > y1):
        k = 180
    else:
        k = 0

    theta = math.atan(-zj / (y1 - yj)) * 180 / math.pi + k

    return theta

def update_angles(theta1, theta2, theta3):
    theta_1 = theta1
    theta_2 = theta2
    theta_3 = theta3

def update_xyz(x, y, z):
    pos_x = x
    pos_y = y
    pos_z = z

def move_xyz(x, y, z):

    angle = []
    angle = inverse_kinematics(x, y, z)
    if(angle[3] == -1):
        print ("Wspolrzedne poza zasiegiem!\n")
        return

    step = []
    step = get_steps(angle[0], angle[1], angle[2])
#TODO zrobic graniczne wartosci
    move_motors(step[0], step[1], step[2])
    update_xyz(x, y, z)
    update_angles(angle[0], angle[1], angle[2])

    print("Obliczone katy:\n", angle[0], "\n", angle[1], "\n", angle[2], "\n")
    print("Obliczone kroki:\n", step[0], "\n", step[1], "\n", step[2], "\n")
    print("Powinnien sie ruszyc...\n")
