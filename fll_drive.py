from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
import umath

hub = PrimeHub()
WHEEL_DIAMETER_MM = 62.0
AXLE_TRACK_MM = 96.0
left_motor = Motor(Port.A)
right_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE)

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER_MM, axle_track=AXLE_TRACK_MM)

# calibrat cu forward(100) = 100 cm
DEG_PER_CM = 3600.0 / (umath.pi * WHEEL_DIAMETER_MM)  # cate grade face motorul pe 1 cm

LOOP_MS = 10
MAX_MOTOR_SPEED = 900
SETTLE_MS = 50

# castiguri pe secunda (independente de LOOP_MS)
TURN_KP = 4.0
TURN_KI = 0.5
TURN_KD = 0.09
TURN_MAX_SPEED = 550
TURN_MIN_SPEED = 40
TURN_MIN_ZONE = 5.0 # sub 5 grade eroare nu mai fortam viteza minima
TURN_ACCEL = 2000 # cat de repede creste viteza la pornire
TURN_TOLERANCE = 1.0
TURN_SETTLED_RATE = 15.0 # sub asta consideram ca nu se mai roteste
TURN_SETTLE_LOOPS = 3
TURN_I_ZONE = 8.0 # integrala porneste doar sub 8 grade eroare
TURN_I_MAX = 40.0
TURN_RATE_FILTER = 0.4 # 0..1, mai mic = viteza masurata mai netezita

FWD_KP = 2.0
FWD_KD = 0.02
FWD_MAX_SPEED = 500
FWD_MIN_SPEED = 100
FWD_ACCEL = 1200
FWD_DECEL = 1000
FWD_MAX_CORRECTION = 150

# unde ar trebui sa se uite robotul nu se reseteaza intre miscari
target_heading = 0.0


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def sign(x):
    return -1 if x < 0 else 1


# timpul real dintre doua bucle in secunde
def get_dt(loop_timer):
    dt = loop_timer.time() / 1000.0
    loop_timer.reset()
    return dt if dt > 0.002 else LOOP_MS / 1000.0


# inceputul fiecarei misiuni cu robotul nemiscat
def start_run(timeout_ms=3000):
    global target_heading
    left_motor.hold()
    right_motor.hold()
    sw = StopWatch()
    # asteapta giroscopul sa se calibreze
    while not hub.imu.ready() and sw.time() < timeout_ms:
        wait(10)
    hub.imu.reset_heading(0)
    target_heading = 0.0


# dupa aliniere de perete
def set_heading(heading=0.0):
    global target_heading
    hub.imu.reset_heading(heading)
    target_heading = heading


# se roteste pana la un unghi absolut (0 = cum a pornit din baza)
def turn_to(heading, max_speed=None, timeout_ms=None):
    global target_heading
    target_heading = heading
    if not max_speed:
        max_speed = TURN_MAX_SPEED

    error = hub.imu.heading() - heading
    if timeout_ms is None:
        timeout_ms = 1000 + abs(error) * 12 # sa nu ramana blocat daca se agata

    timer = StopWatch()
    loop_timer = StopWatch()
    prev_error = error
    rate = 0.0
    integral = 0.0
    power = 0.0
    settled = 0

    while timer.time() < timeout_ms:
        wait(LOOP_MS)
        dt = get_dt(loop_timer)

        error = hub.imu.heading() - heading
        crossed = error * prev_error < 0 # a trecut de tinta in bucla asta
        raw_rate = (error - prev_error) / dt
        prev_error = error

        # heading-ul vine in trepte, fara netezire derivata da smucituri
        rate += TURN_RATE_FILTER * (raw_rate - rate)

        # iese doar daca e pe unghi SI s-a oprit altfel trece de tinta
        if abs(error) <= TURN_TOLERANCE and abs(rate) < TURN_SETTLED_RATE:
            settled += 1
            if settled >= TURN_SETTLE_LOOPS:
                break
        else:
            settled = 0

        # dupa ce trece de tinta integrala veche l-ar impinge si mai departe
        if crossed or abs(error) >= TURN_I_ZONE:
            integral = 0.0
        else:
            i_lim = TURN_I_MAX / TURN_KI
            integral = clamp(integral + error * dt, -i_lim, i_lim)

        cmd = TURN_KP * error + TURN_KI * integral + TURN_KD * rate
        cmd = clamp(cmd, -max_speed, max_speed)

        # viteza minima doar departe de tinta, aproape PID-ul termina incet
        if abs(error) > TURN_MIN_ZONE and sign(cmd) == sign(error) and abs(cmd) < TURN_MIN_SPEED:
            cmd = TURN_MIN_SPEED * sign(error)

        # pornire lina contra patinarii frana imediata
        if abs(cmd) > abs(power):
            step = TURN_ACCEL * dt
            power = clamp(cmd, power - step, power + step)
        else:
            power = cmd

        left_motor.run(power)
        right_motor.run(-power)

    left_motor.hold()
    right_motor.hold()
    wait(SETTLE_MS)


# viraj relativ la unde se uita robotul ACUM (pozitiv = dreapta)
def turn(degrees, max_speed=None, timeout_ms=None):
    turn_to(hub.imu.heading() + degrees, max_speed, timeout_ms)


# stop=False si deceleration=False = miscari legate fara oprire
def forward(distance_cm, speed=FWD_MAX_SPEED, deceleration=True, stop=True, timeout_ms=None):
    target = abs(distance_cm) * DEG_PER_CM
    if target < 1:
        return
    direction = sign(distance_cm)
    max_speed = clamp(abs(speed), FWD_MIN_SPEED, FWD_MAX_SPEED)

    if timeout_ms is None:
        timeout_ms = 1500 + 2000 * target / max_speed

    # fara reset la motoare doar pozitia de start
    left_start = left_motor.angle()
    right_start = right_motor.angle()
    timer = StopWatch()
    loop_timer = StopWatch()
    prev_error = hub.imu.heading() - target_heading

    while timer.time() < timeout_ms:
        wait(LOOP_MS)
        dt = get_dt(loop_timer)

        traveled = abs((left_motor.angle() - left_start + right_motor.angle() - right_start) / 2)
        remaining = target - traveled
        if remaining <= 0:
            break

        # accelereaza la inceput incetineste spre final (v^2 = v0^2 + 2*a*d)
        current_speed = min(max_speed, umath.sqrt(FWD_MIN_SPEED ** 2 + 2 * FWD_ACCEL * traveled))
        if deceleration:
            current_speed = min(current_speed, umath.sqrt(FWD_MIN_SPEED ** 2 + 2 * FWD_DECEL * remaining))

        # tine directia target_heading
        error = hub.imu.heading() - target_heading
        derivative = (error - prev_error) / dt
        prev_error = error
        correction = clamp(FWD_KP * error + FWD_KD * derivative, -FWD_MAX_CORRECTION, FWD_MAX_CORRECTION)

        # lasa loc pentru corectie altfel o roata da in limita si nu mai corecteaza
        current_speed = min(current_speed, MAX_MOTOR_SPEED - abs(correction)) * direction

        left_motor.run(current_speed + correction)
        right_motor.run(current_speed - correction)

    #robot.use_gyro(False)
    #robot.straight(current_speed)
    #robot.use_gyro(True)
        


    if stop:
        left_motor.hold()
        right_motor.hold()
        wait(SETTLE_MS)


# daca atasamentul se agata se opreste in loc sa blocheze tot runul
def run_attachment(motor, speed, angle, then=Stop.HOLD, wait_done=True, timeout_ms=3000):
    motor.run_angle(speed, angle, then=then, wait=False)
    if not wait_done:
        return
    sw = StopWatch()
    while not motor.done():
        if motor.stalled() or sw.time() > timeout_ms:
            motor.hold()
            break
        wait(10)


# duty_limit = forta limitata sa nu forteze piesele la capat
def home_attachment(motor, speed=200, duty_limit=40):
    motor.run_until_stalled(-abs(speed), then=Stop.HOLD, duty_limit=duty_limit)
    motor.reset_angle(0)