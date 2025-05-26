# LUMASTIR

A stirring reactor for Opentrons that allows for computer vision from the side windows!

## LED control: 

These are 3.3V LEDs ([Adafruit White LED Backlight Module - Small 12mm x 40mm](https://www.adafruit.com/product/1626))

We are using GPIO pins on Pi Zero 2W. GPIO 17 (LED 1), GPIO 18 (LED 2), GPIO 27 (LED 3). 

They share the same ground wire connected to the Ground Pin next to GPIO 10.

## Motor control: 

MOTORs: These are 5V CPU fans ([HighPi Pro 5V Cooling Fan](https://www.pishop.ca/product/highpi-pro-5v-cooling-fan/)) controlled using the I2C interface of Pi Zero 2W (need to be enabled), via the PCA9658 board ([Adafruit 16-Channel 12-bit PWM/Servo Driver - I2C interface](https://www.adafruit.com/product/815)).

We are using Servo Motor pins (channels range from 0 to 15): channel 4 (MOTOR 1), channel 8 (MOTOR 2), channel 12 (MOTOR 3), channel 3 (MOTOR 4), channel 7 (MOTOR 5), channel 11 (MOTOR 6).

## Power connections:

5V Power is provided through the POGO pins on the base of this reactor. These pins are connected to a 5V power supply when in contact with the HOTSEAT dock in the 96-well plate form factor.

Link to the HOTSEAT dock: to be added (#TODO)

The 5V power is directly supplied to the battery management system (BMS, [Adafruit Powerboost 1000 Charger](https://www.adafruit.com/product/2465)) to keep charging when docked. The BMS is connected to a LiPo battery ([EEMB 3.7V Li-ion 963450 1800mAh Rechargeable LiPo Battery](https://www.amazon.ca/EEMB-Battery-1800mAh-Rechargeable-Connector/dp/B08ZCQXFX4?th=1)) by a fuel gauge ([Adafruit LC709203F LiPoly / LiIon Fuel Gauge and Battery Monitor](https://www.adafruit.com/product/4712)). The fuel gauge allows for the disconnection of the battery via software when deemed necessary.

The Pi Zero 2W is powered by the USB power-output interface, and connected to PCA9658 via the I2C connections, including 3.3V power, SDA (Serial Data Line, GPIO2 on Pi Zero 2W) and SCL (Serial Clock Line, GPIO3 on Pi Zero 2W).

A hardware switch is added to either (a) turn off the BMS by connecting EN to GND, and disconnecting GND from the Ground of PCA9658, therefore turning off both the Pi and motor driver, or (b) the reverse, turning on the Pi and motor driver.
