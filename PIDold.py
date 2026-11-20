from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import umath

hub = PrimeHub()
left_motor = Motor(Port.A)
right_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

robot = DriveBase(left_motor, right_motor, 62, 120)


TURN_KP = 4.0
TURN_KI = 0.01
TURN_KD = 4.5

#4.0, 0.01, 4.5

FWD_KP = 2.0
FWD_KI = 0.0
FWD_KD = 1.0

TURN_TOLERANCE = 1
FWD_TOLERANCE  = 5
MIN_TURN_POWER = 125
MAX_TURN_POWER = 750
MAX_FWD_SPEED = 600 #600
INTEGRAL_CAP = 50
LOOP_MS = 20
SETTLE_MS = 100

#temp:
FADE_THRESHOLD = 15
STABLE_COUNT_NEEDED = 2
MAX_LOOPS = 3000



def angle_error(current, target):
    c  = umath.radians(current)
    t  = umath.radians(target)
    cx, cy = umath.cos(c), umath.sin(c)
    tx, ty = umath.cos(t), umath.sin(t)
    return umath.degrees(umath.atan2(tx*cy - ty*cx, tx*cx + ty*cy))



def turn_bad(degrees, extraSpeed=0, min_power=MIN_TURN_POWER):
    hub.imu.reset_heading(0)
    wait(100)

    speed_scale = min(1.0, abs(degrees) / 45.0)
    max_power = max(MIN_TURN_POWER, MAX_TURN_POWER * speed_scale)

    # Base min_power — lower than before
    if abs(degrees) <= 15:
        base_min = 50
    else:
        base_min = 80 + ((abs(degrees) - 16) / 165.0) * 40  # caps ~100 at 180

    integral = 0.0
    prev_error = 0.0
    stable_count = 0
    loops = 0

    while True:
        current = hub.imu.heading()
        error = angle_error(current, degrees)

        if abs(error) <= TURN_TOLERANCE:
           break


        loops += 1
        if loops >= MAX_LOOPS:
            break

        integral += error
        integral = max(-INTEGRAL_CAP, min(INTEGRAL_CAP, integral))
        derivative = error - prev_error
        prev_error = error

        power = TURN_KP * error + TURN_KI * integral + TURN_KD * derivative
        power = max(-max_power, min(max_power, power))

        
        if abs(error) < FADE_THRESHOLD:
            fade = abs(error) / FADE_THRESHOLD
            current_min = base_min * fade
        else:
            current_min = base_min

        
        if   0 < power < current_min:   power =  current_min
        elif 0 > power > -current_min:  power = -current_min

        left_motor.run(power)
        right_motor.run(-power)

        wait(LOOP_MS)

    left_motor.brake()
    right_motor.brake()
    wait(SETTLE_MS)


def turn(degrees, extraSpeed = 0, min_power = MIN_TURN_POWER):
    hub.imu.reset_heading(0)
    wait(100)

    speed_scale = min(1.0, abs(degrees) / 45.0)
    max_power = max(MIN_TURN_POWER, (MAX_TURN_POWER) * speed_scale)
    if abs(degrees) <= 15:
        min_power = 50
    else:
        min_power = 75 + ((abs(degrees) - 16) / 165.0) * 75
    #60 + (abs(degrees) / 180.0) * 50

    integral   = 0.0
    prev_error = 0.0

    while True:
        current = hub.imu.heading()
        error   = angle_error(current, degrees)

        if abs(error) <= TURN_TOLERANCE:
            break

        integral  += error
        integral   = max(-INTEGRAL_CAP, min(INTEGRAL_CAP, integral))
        derivative = error - prev_error
        prev_error = error

        power = TURN_KP*error + TURN_KI*integral + TURN_KD*derivative
        power = max(-max_power, min(max_power, power))

        if   0 < power < MIN_TURN_POWER:  power =  min_power
        elif 0 > power > -MIN_TURN_POWER: power = -min_power

        left_motor.run(power)
        right_motor.run(-power)

        wait(LOOP_MS)

    left_motor.brake()
    right_motor.brake()
    wait(SETTLE_MS)

#min_speed = 150
#speed = 600
def movement(distance_cm, speed=MAX_FWD_SPEED, accel_fraction=0.4, decel_fraction=0.25, min_speed=150):
    wheel_diameter_mm = 63.1
    circumference_mm = umath.pi * wheel_diameter_mm
    target_deg = (distance_cm * 10 / circumference_mm) * 360

    direction = 1 if distance_cm > 0 else -1
    speed = abs(speed)

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    hub.imu.reset_heading(0)
    wait(20)

    integral   = 0.0
    prev_error = 0.0

    accel_end   = abs(target_deg) * accel_fraction
    decel_start = abs(target_deg) * (1.0 - decel_fraction)

    while True:
        left_deg  = left_motor.angle()
        right_deg = right_motor.angle()
        traveled  = abs((left_deg + right_deg) / 2)

        if traveled >= abs(target_deg) - FWD_TOLERANCE:
            break

        if traveled < accel_end:
            ramp = traveled / accel_end
            current_speed = min_speed + (speed - min_speed) * ramp
        elif traveled >= decel_start:
            remaining = abs(target_deg) - traveled
            ramp = remaining / (abs(target_deg) * decel_fraction)
            current_speed = min_speed + (speed - min_speed) * ramp
        else:
            current_speed = speed

        current_speed = current_speed * direction

        heading = hub.imu.heading()
        error = -heading

        integral += error
        integral = max(-INTEGRAL_CAP, min(INTEGRAL_CAP, integral))
        derivative = error - prev_error
        prev_error = error

        correction = FWD_KP * error + FWD_KI * integral + FWD_KD * derivative
        correction = max(-100, min(100, correction))

        left_speed = max(-MAX_FWD_SPEED, min(MAX_FWD_SPEED, current_speed - correction))
        right_speed = max(-MAX_FWD_SPEED, min(MAX_FWD_SPEED, current_speed + correction))

        left_motor.run(left_speed)
        right_motor.run(right_speed)

        wait(LOOP_MS)

    left_motor.hold()
    right_motor.hold()
    wait(SETTLE_MS)

def forward(distance_cm, speed=MAX_FWD_SPEED, deceleration = False):
    wheel_diameter_mm = 62
    circumference_mm = umath.pi * wheel_diameter_mm
    target_deg = (distance_cm * 10 / circumference_mm) * 360

    direction = 1 if distance_cm > 0 else -1
    speed = abs(speed) * direction

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    hub.imu.reset_heading(0)
    wait(20)

    integral   = 0.0
    prev_error = 0.0

    while True:
        left_deg  = left_motor.angle()
        right_deg = right_motor.angle()
        traveled  = (left_deg + right_deg) / 2

        if abs(traveled) >= abs(target_deg) - FWD_TOLERANCE:
            break

        if abs(traveled) >= 0.8 * abs(target_deg) and deceleration:
            slow_speed = max(150, int(abs(speed) * 0.4))
            current_speed = slow_speed * direction
        else:
            current_speed = speed

        heading = hub.imu.heading()
        error = -heading

        integral += error
        integral = max(-INTEGRAL_CAP, min(INTEGRAL_CAP, integral))
        derivative = error - prev_error
        prev_error = error

        correction = FWD_KP*error + FWD_KI*integral + FWD_KD*derivative
        correction = max(-100, min(100, correction))

        left_speed = max(-MAX_FWD_SPEED, min(MAX_FWD_SPEED, current_speed - correction))
        right_speed = max(-MAX_FWD_SPEED, min(MAX_FWD_SPEED, current_speed + correction))

        left_motor.run(left_speed)
        right_motor.run(right_speed)

        wait(LOOP_MS)

    left_motor.brake()
    right_motor.brake()
    wait(SETTLE_MS)


def run_attachment(motor, speed, angle, then=Stop.HOLD, wait=True):
    motor.run_angle(speed, angle, then=then, wait=wait)

def home_attachment(motor, speed=200):
    motor.run_until_stalled(-speed, then=Stop.HOLD)
    motor.reset_angle(0)

def drive(cm_dist):
    mm_dist = cm_dist * 10
    robot.straight(mm_dist)