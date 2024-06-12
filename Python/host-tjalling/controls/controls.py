import time
import RPi.GPIO as GPIO

class Led:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 10)
        self.pwm.start(0)
        self.is_pwm_running = True


    def set_brightness(self, brightness):
        if brightness > 0:
            if not self.is_pwm_running:
                self.pwm.start(brightness)
                self.is_pwm_running = True
            print("setting brightness to: ", brightness)
            self.pwm.ChangeDutyCycle(brightness)
        else:
            if self.is_pwm_running and brightness == 0:
                print("stopping pwm")
                self.pwm.stop()
                self.is_pwm_running = False

def control_rgb_led(red_brightness, green_brightness, blue_brightness):
    GPIO.setmode(GPIO.BCM)
    red_led = Led(22)
    green_led = Led(27)
    blue_led = Led(17)

    print("red brightness: ", red_brightness)
    print("green brightness: ", green_brightness)
    print("blue brightness: ", blue_brightness)

    red_led.set_brightness(red_brightness)
    green_led.set_brightness(green_brightness)
    blue_led.set_brightness(blue_brightness)