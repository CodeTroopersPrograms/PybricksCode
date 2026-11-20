from pybricks.hubs import PrimeHub
from pybricks.parameters import Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.tools import wait

from fll_drive import forward, run_attachment, start_run, turn

hub = PrimeHub()
l_arm = Motor(Port.E)
r_arm = Motor(Port.B)
color_sensor = ColorSensor(Port.F)

def main():
    while True:
        hsv = color_sensor.hsv()
        if 200 <= hsv.h <= 230 and 50 <= hsv.s <= 95 and 20 <= hsv.v <= 50:
            print("blue")
            print(color_sensor.color())
            hub.speaker.beep(frequency=400, duration=500)
            forward(-55)
            turn(-47)
            forward(-20)
            turn(46)
            forward(-8)
            run_attachment(r_arm, 100, 100, wait_done=True)
            forward(8)
            run_attachment(r_arm, 100, -90)
                
        elif 0 <= hsv.h <= 12 and hsv.s > 50:
            #0 12
            print("orange")
            hub.speaker.beep(frequency=400, duration=500)
            turn(90)
            forward(100)
            
        elif 80 <= hsv.h <= 95 and 50 <= hsv.s <= 70: # Verde
            # a doua dunga din partea dreapta, sa nu treaca peste
            print("green")
            start_run() #calibreaza roti + gyro
            hub.speaker.beep(frequency=400, duration=500)
            forward(-7)
            turn(45)
            forward(-30)
            run_attachment(l_arm, 35, -69, then=Stop.HOLD)
            forward(10)
            forward(25)
            turn(45)
            forward(20)
            run_attachment(l_arm, 80, -100, then=Stop.HOLD)
            wait(5)
            forward(-75)
            forward(70)


            
        elif 50 <= hsv.h <= 55 and 67 <= hsv.s <= 71:
            #2 negru 2
            print("yellow")
            hub.speaker.beep(frequency=400, duration=500)
            run_attachment(r_arm, 1000, 45, then=Stop.HOLD, wait_done=True)
            run_attachment(r_arm, 2000, 180, then=Stop.HOLD)
            forward(10)
            run_attachment(r_arm, 80, -180, then=Stop.HOLD)
            forward(-12)
            run_attachment(r_arm, 1800, 180, then=Stop.HOLD)

            
        elif 270 <= hsv.h <= 335 and hsv.s >= 40 and hsv.v >= 20:
            print("magenta")
            print(color_sensor.color())
            hub.speaker.beep(frequency=400, duration=500)
            forward(-51)
            forward(51)
           
        elif 0 <= hsv.s <= 25 and hsv.v >= 70:
            print("WHITE")
            hub.speaker.beep(frequency=400, duration=500)
            voltage = hub.battery.voltage()

            # Typical voltage range for a Li-ion hub (adjust if using AA/rechargeable packs)
            min_voltage = 6500  # 6.5V (empty)
            max_voltage = 8400  # 8.4V (full)

            battery_percent = (voltage - min_voltage) * 100 / (max_voltage - min_voltage)
            battery_percent = max(0, min(100, battery_percent))  # Clamp between 0 and 100%

            print(f"Voltage: {voltage} mV")
            print(f"Battery level: {battery_percent:.1f}%")


if __name__ == "__main__":
    main()
