# # Code by 11138472 Cheuk Hei Matthew Tam
# Avionics tutorial 2.1C
# Importing Libraies
import board
import digitalio
import time

#  LED setup
greenled = digitalio.DigitalInOut(board.GP20)
greenled.direction = digitalio.Direction.OUTPUT

yellowled = digitalio.DigitalInOut(board.GP19)
yellowled.direction = digitalio.Direction.OUTPUT

redled = digitalio.DigitalInOut(board.GP18)
redled.direction = digitalio.Direction.OUTPUT

# Button setup
button = digitalio.DigitalInOut(board.GP21) 
button.switch_to_input(pull=digitalio.Pull.DOWN)  # Memorize this

#  Main Loop
while True:
    greenled.value = True
    
    if button.value:  # If button is at 1 (pull down means it is always at 0 unless connected) 
        time.sleep(3)
        greenled.value = False 
        yellowled.value = True
        time.sleep(2)
        yellowled.value = False
        redled.value = True
        time.sleep(3)
        yellowled.value = True
        time.sleep(2)
        redled.value = False
        yellowled.value = False
        greenled.value = True 