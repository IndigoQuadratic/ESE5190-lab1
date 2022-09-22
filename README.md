# ESE5190-lab1
real-time visualizer
import time
import board
import digitalio
import usb_hid
import analogio
import digitalio
import usb_hid
import time
import board
import busio
import adafruit_apds9960.apds9960
import neopixel
pixels = neopixel.NeoPixel(board.NEOPIXEL, 1)
i2c = board.STEMMA_I2C()
sensor = adafruit_apds9960.apds9960.APDS9960(i2c)
sensor.enable_color = True
sensor.color_integration_time = 10
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keyboard_layout_us import KeyboardLayoutUS
from adafruit_hid.keycode import Keycode
keyboard = Keyboard(usb_hid.devices)
keyboard_layout = KeyboardLayoutUS(keyboard)  
sensor.enable_proximity = True
sensor.enable_gesture = True


while True:
    gesture = sensor.gesture()
    if gesture == 0x01:
        keyboard.press(82)
        keyboard.release_all()
    elif gesture == 0x02:
        keyboard.press(81)
        keyboard.release_all()
    elif gesture == 0x03:
        keyboard.press(80)
        keyboard.release_all()
    elif gesture == 0x04:
        keyboard.press(79)
        keyboard.release_all()
        
    r, g, b, c = sensor.color_data
    if c == 0:
        break
    time.sleep(0.01)
