# # Code by 11138472 Cheuk Hei Matthew Tam
# Avionics tutorial 2.1
# Importing Libraies
import board
import digitalio
import time

#  LED setup
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Button setup
button = digitalio.DigitalInOut(board.GP21) 
button.switch_to_input(pull=digitalio.Pull.DOWN)  # Memorize this

#  Main Loop
while True:
    if button.value:  # If button is at 1 (pull down means it is always at 0 unless connected) 
        led.value = True
        time.sleep(1)
        led.value = False
        time.sleep(1)
        print("Blinking")
        #  Blink the pico's LED for at 1 Hz 