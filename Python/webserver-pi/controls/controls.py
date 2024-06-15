import RPi.GPIO as GPIO
import smbus


class Led:
    
    """
    Initialize the Led object.

    Args:
        pin (int): The GPIO pin number for the LED.
    """
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 10000)
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
        

# this is not yet working no i2c device found yet so need fixing(maybe unnececary as we probably can use load from servo)
class Encoder:
    """
    Initialize the Encoder object.
    """
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.bus = smbus.SMBus(1)  # Use I2C bus 1
        self.address = 0x36  # Address of the AS5600 encoder
        self.direction_pin = 22  # BCM pin for the direction pin
        GPIO.setup(self.direction_pin, GPIO.IN)

    """
    Read the angle from the AS5600 encoder.

    Returns:
        int: The angle value.
    """
    def read_angle(self):  
        angle = self.bus.read_word_data(self.address, 0x0E)
        return angle

    """
    Read the direction from the direction pin.

    Returns:
        int: The direction value.
    """
    def read_direction(self):
        
        direction = GPIO.input(self.direction_pin)
        return direction
