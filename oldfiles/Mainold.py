import PID
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
l_arm = Motor(Port.E)
r_arm = Motor(Port.B)
color_sensor = ColorSensor(Port.F)

def main():
    cnt = 0
    cntG = 1
    while True:
        hsv = color_sensor.hsv()
        if 200 <= hsv.h <= 230 and 50 <= hsv.s <= 95 and 20 <= hsv.v <= 50:
            print("blue")
            print(color_sensor.color())
            hub.speaker.beep(frequency=400, duration=500)
            if cnt == 0:
                #negru
                PID.run_attachment(r_arm, 500, -120, wait=False)
                PID.movement(70)
                PID.turn(20)
                PID.run_attachment(r_arm, 500, 40)
                wait(100)
                PID.turn(-13) 
                #PID.run_attachment(r_arm, 500, -120)
                PID.forward(3)
                PID.run_attachment(l_arm, 500, 30)
                PID.turn(-14)
                wait(100)
                PID.forward(-1.5)
                PID.turn(-84)
                PID.forward(-40)
                PID.run_attachment(r_arm, 900, -60)
                PID.run_attachment(l_arm, 900, 100)
                PID.turn(45)
                PID.run_attachment(r_arm, 900, 65)
                PID.forward(6)
                PID.turn(60)
                cnt = 1
            elif cnt == 1:
                #negru 1
                PID.movement(25, speed=600, min_speed=150)
                PID.turn(-43)
                PID.movement(22, speed=600, min_speed=150)
                PID.run_attachment(r_arm, 1000, 200)
                PID.run_attachment(l_arm, 500, 160)
                PID.movement(-10)
                PID.run_attachment(l_arm, 500, -120)
                PID.turn(-24)
                PID.movement(5, speed=500, min_speed=150)
                #PID.turn(2)
                PID.run_attachment(l_arm, 500, 160)
                PID.turn(-30)
                PID.movement(-20)
                PID.turn(-120)
                PID.movement(40)
                cnt = 0
                
        elif 0 <= hsv.h <= 12 and hsv.s > 50:
            #0 12
            print("orange")
            hub.speaker.beep(frequency=400, duration=500)
            if cntG == 0:
                #negru 1-2
                PID.movement(74, speed=900, min_speed=800)
                #wait(500)
                PID.movement(-74)
                cntG = 1
            else:
                #negru 2-2
                PID.forward(46, 300)
                PID.movement(-3)
                PID.turn(5)
                PID.run_attachment(l_arm, 300, 45)
                #PID.forward(-12, deceleration=True)
                PID.movement(-47, speed=600, min_speed=500)
                cntG = 0
        elif 80 <= hsv.h <= 95 and 50 <= hsv.s <= 70:
            #2 negru 1
            print("green")
            hub.speaker.beep(frequency=400, duration=500)
            PID.movement(73.5, speed=500, min_speed=400)
            PID.turn(-35)
            PID.run_attachment(r_arm, 500, -88)
            PID.movement(14, speed=200, min_speed=150)
            PID.movement(-4.5, speed=200, min_speed=150)
            PID.run_attachment(l_arm, 500, 120)
            PID.run_attachment(r_arm, 500, 100)
            PID.movement(3, speed=200, min_speed=150)
            PID.run_attachment(r_arm, 300, -400, wait=False)
            PID.movement(-25)
            PID.turn(60)
            PID.movement(-50, min_speed=400)

        elif 50 <= hsv.h <= 55 and 67 <= hsv.s <= 71:
            #2 negru 2
            print("yellow")
            hub.speaker.beep(frequency=400, duration=500)
            PID.run_attachment(l_arm, 500, -100)
            PID.forward(88.5)
            PID.turn(88.1)
            PID.run_attachment(l_arm, 500, 100)
            PID.run_attachment(r_arm, 500, -250)
            PID.forward(10, speed=100)
            PID.run_attachment(l_arm, 200, -50)
            PID.run_attachment(r_arm, 200, 250)
            PID.forward(-10, 200)
            PID.run_attachment(l_arm, 500, -50)
            PID.turn(100)
            PID.movement(75, speed=900, min_speed=800)
            """
            PID.movement(61)
            PID.turn(65)
            PID.movement(-3, 100)
            PID.run_attachment(l_arm, 100, 120)
            PID.movement(3, 100)
            PID.movement(-5, 250)
            PID.turn(-20)
            PID.movement(-2, 100)
            PID.turn(-20)
            """
        elif 270 <= hsv.h <= 335 and hsv.s >= 40 and hsv.v >= 20:
            print("magenta")
            print(color_sensor.color())
            #penultima neagra
            hub.speaker.beep(frequency=400, duration=500)
            PID.movement(98, speed=700, min_speed=600)
            PID.turn(60)
            PID.movement(11, speed=600, min_speed=150)
            PID.run_attachment(l_arm, 500, -70)
            PID.movement(-18, 200)
            PID.turn(26)
            PID.movement(-14, 200)
            PID.turn(-10)
            PID.run_attachment(r_arm, 500, 1000)
            PID.movement(20, speed=700, min_speed=600)
            PID.turn(-80)
            PID.movement(80,speed=800, min_speed=700)
        elif 0 <= hsv.s <= 25 and hsv.v >= 70:
            print("WHITE")
            hub.speaker.beep(frequency=400, duration=500)
            PID.movement(30, speed=600, min_speed=500)
            PID.turn(50)
            PID.movement(12, speed=600, min_speed=500)
            PID.turn(15)
            PID.movement(5, speed=600, min_speed=700)
            PID.turn(10)


           
            
if __name__ == "__main__":
    main()
