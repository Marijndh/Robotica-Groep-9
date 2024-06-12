import time
import RPi.GPIO as GPIO
import threading

class Led:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 10000)
        self.pwm.start(0)
        self.is_pwm_running = True
        self.red_brightness = 0
        self.green_brightness = 0
        self.red_brightness = 0


    def set_brightness(self, brightness):
        if brightness > 0:
            if not self.is_pwm_running:
                self.pwm.start(brightness)
                self.is_pwm_running = True
            self.pwm.ChangeDutyCycle(brightness)
            #GPIO.output(self.pin, GPIO.HIGH)
        else:
            if self.is_pwm_running and brightness == 0:
                self.pwm.stop()
                self.is_pwm_running = False

class Leds:
    def __init__(self):
        self.red_brightness = 0
        self.green_brightness = 0
        self.blue_brightness = 0
        GPIO.setmode(GPIO.BCM)
        self.red_led = Led(22)
        self.green_led = Led(27)
        self.blue_led = Led(17)

    def control_rgb_led(self,red_brightness, green_brightness, blue_brightness):
        self.red_brightness = red_brightness
        self.green_brightness = green_brightness
        self.blue_brightness = blue_brightness

        print("red brightness: ", red_brightness)
        print("green brightness: ", green_brightness)
        print("blue brightness: ", blue_brightness)
        self.red_led.set_brightness(self.red_brightness)
        self.green_led.set_brightness(self.green_brightness)
        self.blue_led.set_brightness(self.blue_brightness)

        # def keep_alive():
        #     try:
        #         while True:  # keep the script running
        #             #time.sleep(1)  # sleep for 1 second at a time
        #             self.red_led.set_brightness(self.red_brightness)
        #             self.green_led.set_brightness(self.green_brightness)
        #             self.blue_led.set_brightness(self.blue_brightness)
        #     except KeyboardInterrupt:  # allow the script to be stopped with Ctrl+C
        #         GPIO.cleanup()  # cleanup GPIO on Ctrl+C exit

        # Start the keep_alive function in a new thread
        #threading.Thread(target=keep_alive).start()
    