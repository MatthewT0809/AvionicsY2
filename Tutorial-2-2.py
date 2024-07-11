# # Code by 11138472 Cheuk Hei Matthew Tam
# Avionics tutorial 2.2
# Importing Libraies
import board
import digitalio
import time

#  LED setup
greenled = digitalio.DigitalInOut(board.GP20)
greenled.direction = digitalio.Direction.OUTPUT
greenledtimer = time.monotonic()

yellowled = digitalio.DigitalInOut(board.GP19)
yellowled.direction = digitalio.Direction.OUTPUT
yellowledtimer = time.monotonic() 

# Variable
greenBlinkRate = 1
yellowBlinkRate = 0.2


#  Main Loop
while True: 
    if time.monotonic() > greenledtimer+greenBlinkRate: 
        greenled.value = not greenled.value
        greenledtimer = time.monotonic() 
    if time.monotonic() > yellowledtimer+yellowBlinkRate:
        yellowled.value = not yellowled.value
        yellowledtimer = time.monotonic() 