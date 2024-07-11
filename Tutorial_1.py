# # Code by 11138472 Cheuk Hei Matthew Tam
# Avionics tutorial 1.2
# Importing Libraies
import board
import digitalio
import time

#  LED setup
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

#  Main Loop
while True:
    led.value = True
    time.sleep(1)
    led.value = False
    time.sleep(1)
    print("Blinking")
    #  Blink the pico's LED for at 1 Hz 