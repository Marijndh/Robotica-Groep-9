import RPi.GPIO as GPIO
import smbus
import time


class Led:
    
    """
    Initialize the Led object.

    Args:
        pin (int): The GPIO pin number for the LED.
    """
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 5000)
        self.pwm.start(0)
        self.is_pwm_running = True
        self.red_brightness = 0
        self.green_brightness = 0
        self.red_brightness = 0
    
    """
    Set the brightness of the LED.

    Args:
        brightness (float): The brightness value between 0 and 100.
    """
    def set_brightness(self, brightness):
        if brightness > 0:
            if not self.is_pwm_running:
                self.pwm.start(brightness)
                self.is_pwm_running = True
            self.pwm.ChangeDutyCycle(brightness)
        else:
            if self.is_pwm_running and brightness == 0:
                self.pwm.stop()
                self.is_pwm_running = False


class Leds:
    
    """
    Initialize the Leds object.
    """
    def __init__(self):    
        self.red_brightness = 0
        self.green_brightness = 0
        self.blue_brightness = 0
        GPIO.setmode(GPIO.BCM)
        self.red_led = Led(22)
        self.green_led = Led(27)
        self.blue_led = Led(17)
    
    """
    Control the RGB LED.

    Args:
        red_brightness (float): The brightness value for the red LED between 0 and 100.
        green_brightness (float): The brightness value for the green LED between 0 and 100.
        blue_brightness (float): The brightness value for the blue LED between 0 and 100.
    """
    def control_rgb_led(self, red_brightness, green_brightness, blue_brightness):
        
        self.red_brightness = red_brightness
        self.green_brightness = green_brightness
        self.blue_brightness = blue_brightness

        print("red brightness: ", red_brightness)
        print("green brightness: ", green_brightness)
        print("blue brightness: ", blue_brightness)
        self.red_led.set_brightness(self.red_brightness)
        self.green_led.set_brightness(self.green_brightness)
        self.blue_led.set_brightness(self.blue_brightness)
        

"""
Initialize the UltrasonicSensor object.
"""
class UltrasonicSensor:
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.trigger_pin = 23  # BCM pin for the trigger
        self.echo_pin = 24  # BCM pin for the echo
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)

    """
    Read the distance from the HC-SR04 sensor.

    Returns:
        float: The distance value in cm.
    """
    def read_distance(self):
        print("Reading distance...")
        # Send a 10us pulse to the trigger pin
        GPIO.output(self.trigger_pin, True)
        start_reading_time = time.time()
        while time.time() - start_reading_time < 0.000004:
            pass
        GPIO.output(self.trigger_pin, False)

        # Wait for the echo pin to go high
        while GPIO.input(self.echo_pin) == 0 and time.time() - start_reading_time < 2:
            start_time = time.time()
        
        #print("pin was high")

        # Wait for the echo pin to go low
        while GPIO.input(self.echo_pin) == 1 and time.time() - start_reading_time < 3:
            end_time = time.time()
        #print("done reading pulses now calculating distance...")
        if 'end_time' in locals():
            # Calculate the duration of the echo pulse
            pulse_duration = end_time - start_time

            # The speed of sound is 34300 cm/s, and the echo pulse travelled the
            # distance to the object and back, so we divide by 2
            distance = (pulse_duration*1000000) / 58
            if distance > 40:
                distance = -1
        else:
            distance = -1
        #print("Distance: ", distance)
        return distance

