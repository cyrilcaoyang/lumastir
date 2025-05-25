import time
import board
import busio
import RPi.GPIO as GPIO
from adafruit_pca9685 import PCA9685

# Configuration
LED_PINS = [17, 18, 27]  # BCM numbering
MOTOR_CHANNELS = [4, 8, 12, 15]  # PCA9685 channels


class MixedController:
    def __init__(self, led_pins, motor_channels):
        # LED Setup (GPIO)
        self.led_pins = led_pins
        GPIO.setmode(GPIO.BCM)
        self.pwm_leds = {}
        for pin in led_pins:
            GPIO.setup(pin, GPIO.OUT)
            self.pwm_leds[pin] = GPIO.PWM(pin, 100)  # 100 Hz frequency
            self.pwm_leds[pin].start(0)

        # Motor Setup (PCA9685)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 500  # 1 kHz for motors
        self.motor_channels = motor_channels

        # Initialize motors to off
        self.stop_all_motors()

    def set_led_brightness(self, pin, brightness):
        """Control LED brightness (0-100%)"""
        if pin in self.pwm_leds:
            self.pwm_leds[pin].ChangeDutyCycle(brightness)

    def set_motor_speed(self, channel, speed):
        """Control motor speed (0-100%)"""
        if channel in self.motor_channels:
            duty_cycle = int(speed * 655.35)  # Convert % to 16-bit value
            self.pca.channels[channel].duty_cycle = min(duty_cycle, 0xFFFF)

    def blink_led(self, pin, duration=0.5):
        """Simple blink function for LEDs"""
        self.set_led_brightness(pin, 100)
        time.sleep(duration)
        self.set_led_brightness(pin, 0)

    def stop_all_motors(self):
        """Stop all connected motors"""
        for channel in self.motor_channels:
            self.pca.channels[channel].duty_cycle = 0

    def cleanup(self):
        """Clean up resources"""
        self.stop_all_motors()
        for pin in self.pwm_leds:
            self.pwm_leds[pin].stop()
        GPIO.cleanup()
        self.pca.deinit()


if __name__ == "__main__":
    try:
        controller = MixedController(LED_PINS, MOTOR_CHANNELS)

        # LED demonstration
        for i in range(0, 5):
            for pin in LED_PINS:
                controller.blink_led(pin)

        # Motor demonstration
        while True:
            for pin in LED_PINS: #[100, 90, 80, 70, 60, 50]:
                controller.set_led_brightness(pin, 100)
            for channel in [4, 8, 12]:
                controller.set_motor_speed(channel, 80)

        controller.set_motor_speed(channel, 0)

    except KeyboardInterrupt:
        controller.cleanup()
        print("Done")
    finally:
        controller.cleanup()
