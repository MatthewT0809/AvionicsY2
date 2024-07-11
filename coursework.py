# Code by 11138472 Cheuk Hei Matthew Tam
# Avionics Coursework 1 (First Class)
# Importing Libraies

import time
import board
import pwmio
import analogio
import digitalio
import storage
import sdcardio
import busio
import random

# === Setup ===

#  UART
uartRx = busio.UART(board.GP16, board.GP17, baudrate=9600, timeout=0)

# SD card
spi = busio.SPI(board.GP10, MOSI=board.GP11, MISO=board.GP12) #10 11 12
cs = board.GP13
sdcard = sdcardio.SDCard(spi, cs)

# Analog Sensor setup
RPMSensor = analogio.AnalogIn(board.A1)
voltSensor = analogio.AnalogIn(board.A0)
currentSensor = analogio.AnalogIn(board.A2)

# LED setup
LED = digitalio.DigitalInOut(board.GP6)
LED.direction = digitalio.Direction.OUTPUT
REDLED = digitalio.DigitalInOut(board.GP7)
REDLED.direction = digitalio.Direction.OUTPUT

# ESC Setup
f_pwm = 50  # [Hz] How much pulse cyclqe per second
ESC = pwmio.PWMOut(board.GP18, frequency=f_pwm)

# Buzzer Setup
buzzer = pwmio.PWMOut(board.GP5, variable_frequency=True)  # Buzzer pin = 17
low_tone_frequency = 1  # [Hz]
high_tone_frequency = 100

# Button Setup
button = digitalio.DigitalInOut(board.GP4)
button.switch_to_input(pull=digitalio.Pull.DOWN)
airSpeedButton = digitalio.DigitalInOut(board.GP2)
airSpeedButton.switch_to_input(pull=digitalio.Pull.DOWN)

# === Variables ===
codeTimePeriod = 1
codeState = 0
codeTime = time.monotonic()
printTime = time.monotonic()

motorTestTime = time.monotonic()
motorTestTimePeriod = 7

buttonTimer = time.monotonic()
buttonCoolDown = 0.5  # Need this cool down (>0.2)

throttle = 0
airSpeedValue = 8

RPMSensorValue = 0
voltSensorValue = 0
currentSensorValue = 0
power = 0
header = 0
axialForce = 0
axialTorque = 0
rpmTemp = 0
currentTemp = 0
voltTemp = 0

RPMArray = []
currentArray = []
voltArray = []
axialTorqueArray = []
axialForceArray = []

sum_rpm = 0
sum_volt = 0
sum_current = 0
sum_axialForce = 0
sum_axialTorque = 0

initial = True

ampTimer = 0
ampTimerCondition = 10
recording = False

sampleNumber = 1
arrayNum = 0

currentSensitivity = 25
currentOffset = 1.62
RPMsensitivity = 3636
voltageOffset = 0.11750240773085388 #0.0844649441
voltageSensitvity = 8.525214519297213 #8.5276752795

tau = 8.0          # Time constant in seconds
sampling_period = 5 # Sampling period in seconds

# Calculate the filter coefficient, alpha
alpha = sampling_period / (tau + sampling_period)
filtered_volt = 0;
filtered_current = 0;
filtered_RPM = 0;
# === Functions ===
def melody():
    notes = [523, 587, 659, 698, 784, 880, 988, 1047]
    durations = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.4]

    # Play the melody
    for note, duration in zip(notes, durations):
        buzzer.frequency = note  # Set buzzer frequency
        buzzer.duty_cycle = 65536 // 2  # 50% duty cycle (volume)
        time.sleep(duration)  # Wait for note duration
        buzzer.duty_cycle = 0  # Turn off the buzzer

def calculatePulseWidth(throttle):
    t_pwmin = 1000
    t_pwmax = 2000
    return t_pwmin + (t_pwmax - t_pwmin) * throttle / 100

def calculateDuty(pwm_width):
    duty_fraction = (pwm_width / 1000000) * f_pwm
    return duty_fraction * 65535

def calculateCurrent(counts):
    b = (counts/(2**16-1))*(3.3)
    return (b/currentSensitivity)+currentOffset

def calculateRPM(counts):
    c = (counts/(2**16-1))*(5)
    return c * RPMsensitivity

def calculateVoltage(counts):
    d = (counts/(2**16-1))*5
    return d * voltageSensitvity + voltageOffset

def printUpdate():
    print(f"Airspeed Setings = {airSpeedValue} m/s")
    print(f"Throttle Precentage = {throttle}%")
    print(f"Axial Force = %.3f Newtons" % axialForce)
    print(f"Axial Torque = %.3f Newton-meters" % axialTorque)
    print(f"Revolution Per Minute = {RPMSensorValue} RPM")
    print(f"Power = {power} Watts")
    print("====")

def throttleUpdate(throttle):
    pw = calculatePulseWidth(throttle)
    ESC.duty_cycle = int(calculateDuty(pw))
    return pw

def buzzerUpdate(duty , freq):
    buzzer.duty_cycle = duty
    buzzer.frequency = freq

def read_uart_complete(uart_device):
    response = bytearray()
    startMessage = False
    timeOutTime = time.monotonic()  # Record the start time
    while True:
        if uart_device.in_waiting > 0:
            char = uart_device.read(1) # Read data until a newline character is found
            if not startMessage:
                if char == b'<':  # Assuming ">" is the last byte "b" character of the string
                    startMessage = True
                    #print("Start message True")
                    continue
            else:
                if char == b'>':  # Assuming ">" is the last byte "b" character of the string
                    break
                response.extend(char)
        if time.monotonic() > timeOutTime + 2:
            print("Timed out")
            break
    return bytes(response).decode('ascii').strip('<>')

def generate_unique_filename():
    current_time = time.localtime()
    timestamp = "{:04d}{:02d}{:02d}_{:02d}{:02d}{:02d}".format(*current_time[:6])
    # Somehow the earilest time the localtime() function can do is 2020 01 01, no problems with that, but just weird
    filename = f"{"/sd/data"}{timestamp}{".csv"}"
    return filename

filename = generate_unique_filename()

with open(filename, "w") as f:
    print("=========================")
    print(
    "Avionics DAS SYSTEM\nWelcome User, this is the serial view. You can access live information display here in this window. "
    "\nWhen you are ready, press the emergency button to begin. "
    f"\nThis operation's saved data name is called ({filename})."
    "\nPlease refer to the user manual for more instructions\n"
    )
    print("=========================")

with open(filename, "a") as f:
    f.write(" Sampling number, Air Speed m/s, ESC Throttle Settings (pulsewidth), Avg RPM, Avg Voltage, Avg Current, Average AxialForce,Avg Axial Torque \n")

# === Main loop ===
melody()
while True:

    ##TESTING##
#   randomTestValue1 = random.uniform(1.0, 10.0)
#   randomTestValue2 = random.uniform(1.0, 10.0)
#   randomVoltage = 10 #random.uniform(0.0, 26.0)
#   randomCurrent = 10 #random.uniform(0.0, 50.0)
#   randomRPM =  10 #random.uniform(0.0, 12000.0)
#   string = "<###,%.4f,%.4f>" % (randomTestValue1,randomTestValue2)
#   uartTx.write(bytes(string,'ascii'))

    # === Emergency button ===
    if button.value and time.monotonic() > buttonTimer + buttonCoolDown:
    # Button pressed and debounce passed?
        buttonTimer = time.monotonic()  # Cooldown reference time
        if codeState:
        # Determine state (1 is running 0 is stopped)
            print("====EMERGENCY STOP ACTIVATED====")
            ESC.duty_cycle = 0
            REDLED.value = True
            buzzerUpdate(0, 1)
        else:
            print("====STARTING/RESUMING====")
            throttleUpdate(throttle)  # Repower the motor
            REDLED.value = False
        codeState = not codeState  # Update state of the code

    # === Main Code ===
    if codeState:
    # If code is in running mode (Established by the button)
        if voltSensorValue < 22:
            buzzerUpdate(32768, 1000)
            REDLED.value = True
        else:
            buzzerUpdate(0, 10)
            REDLED.value = False

        if currentSensorValue <= 20 and recording:
            ampTimer = 0
            recording = False

        elif currentSensorValue > 20 and not recording:
            ampTimer = time.monotonic()
            recording = True

        if time.monotonic() > ampTimer + ampTimerCondition and recording:
            buzzerUpdate(30000, 200)

        if airSpeedValue == 12 and throttle == 80 and arrayNum == 5:
        # At airspeed of 12m/s, throttle is at max
            buzzerUpdate(32768 , 1000)  # A nice little buzz
            LED.value = True
            while not button.value:
                melody()
                print("Eject SD card!, programme ended")
                time.sleep(0.2)

        if airSpeedValue < 12 and airSpeedButton.value and throttle == 80 and arrayNum == 5:
        # Not at final airspeed, airspeed button pressed, throttle at max,
            throttle = 0
            airSpeedValue += 2
            LED.value = False

        if arrayNum == 5 and throttle == 80:
            if time.monotonic() > printTime + 1:
                print("Operation paused until airspeed button is pressed")
                printTime = time.monotonic()
            LED.value = True

        else:  # Throttle and Data Gathering Section
            if (throttle < 80 and arrayNum == 5) or initial:
                throttle += 10
                motorTestTime = time.monotonic()
                pw = throttleUpdate(throttle)
                print("Increasing throttle")
                print("====")

                sum_rpm = sum(RPMArray)/5
                sum_volt = sum(voltArray)/5
                sum_current = sum(currentArray)/5
                sum_axialForce = sum(axialForceArray)/5
                sum_axialTorque = sum(axialTorqueArray)/5
                filtered_volt = alpha * sum_volt + (1 - alpha) * filtered_volt
                filtered_current = alpha * sum_current + (1 - alpha) * filtered_current
                filtered_RPM = alpha * sum_rpm + (1 - alpha) * filtered_RPM
                print(f"first order flitered volt = {filtered_volt}")
                print(f"first order flitered current= {filtered_current}")
                print(f"first order flitered RPM = {filtered_RPM}")
                with open(filename, "a") as f:
                    f.write(
                    "%f, %f m/s, %.0f ms,%.0f RPM,%.3f V,%.3f A,%.3f N,%.4f Nm \n"
                    %(sampleNumber,airSpeedValue, pw, RPMSensorValue, voltSensorValue, currentSensorValue,axialForce,axialTorque)
                    )
                sampleNumber += 1
                #with open("/sd/data.csv","r") as f:
                    #print(f.read())
                initial = False

                RPMArray.clear()
                currentArray.clear()
                voltArray.clear()
                axialTorqueArray.clear()
                axialForceArray.clear()
                arrayNum = 0
            if throttle <= 80 and time.monotonic() = motorTestTime + motorTestTimePeriod or arrayNum < 5:
                data = read_uart_complete(uartRx)
                print(data)
                if data:
                    while True:
                        data1 = read_uart_complete(uartRx)
                        try:
                            a, b, c = data1.split(',')
                           # print(a, b, c)
                        except ValueError:
                            print("Error: Incorrect data format", data1)
                        else:
                            break
                    header = a
                    axialForce = float(b)
                    axialTorque = float(c)

                if time.monotonic() > motorTestTime + 2:
                    rpmTemp = RPMSensor.value
                    voltTemp = voltSensor.value
                    currentTemp = currentSensor.value
                    RPMSensorValue = calculateRPM(rpmTemp)
                    voltSensorValue = calculateVoltage(voltTemp)
                    currentSensorValue = calculateCurrent(currentTemp)
                    power = voltSensorValue*currentSensorValue

                    RPMArray.append(RPMSensorValue)
                    currentArray.append(currentSensorValue)
                    voltArray.append(voltSensorValue)
                    axialTorqueArray.append(axialForce)
                    axialForceArray.append(axialTorque)

                    arrayNum += 1

            if time.monotonic() > codeTime + codeTimePeriod:  # If time since last message recorded is over than 1s (1HZ input requirement) and throttle is less than 80%:
                codeTime = time.monotonic()  # Record the time of current message printing for 1hz input requirement purposes
                printUpdate()
# Write your code here :-)