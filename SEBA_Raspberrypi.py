import time
import RPi.GPIO as GPIO
import BlynkLib
from BlynkTimer import BlynkTimer
from time import sleep
from mfrc522 import MFRC522
import threading

# Blynk Auth Token
BLYNK_AUTH_TOKEN = 'cnaKMPqHbYCbmGAlnS5hc5kU5XL48EwX'

# Initialize Blynk and Timer
blynk = BlynkLib.Blynk(BLYNK_AUTH_TOKEN)
timer = BlynkTimer()

# GPIO setup
GPIO.setmode(GPIO.BOARD)

# Sensor and actuator pins
SMOKE_SENSOR = 12
FLAME_SENSOR = 11
BUZZER_SMOKE = 16
BUZZER_FLAME = 13
LED1 = 40
LED2 = 38
SERVO_PIN = 36  # GPIO16 ?? BOARD
FAN_PIN = 32    # GPIO12 BOARD pin for fan control

# Pins for LDR script (avoid conflicts)
LDR_PIN = 37
LDR_LED1_PIN = 33
LDR_LED2_PIN = 35

GPIO.setup(LDR_LED1_PIN, GPIO.OUT)
GPIO.setup(LDR_LED2_PIN, GPIO.OUT)

MAX_COUNT = 20000  # ??? ???? ?????? ?? ????
MAX_READING = 1023  # ???? ??????? ???? ??? 0 ? 1023

# Setup pins
GPIO.setup(SMOKE_SENSOR, GPIO.IN)
GPIO.setup(FLAME_SENSOR, GPIO.IN)
GPIO.setup(BUZZER_SMOKE, GPIO.OUT)
GPIO.setup(BUZZER_FLAME, GPIO.OUT)
GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(FAN_PIN, GPIO.OUT)

# Servo setup
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def set_angle(angle):
    duty = 2 + (angle / 18)
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(SERVO_PIN, False)
    pwm.ChangeDutyCycle(0)

def fan_on():
    GPIO.output(FAN_PIN, GPIO.HIGH)
    print("Fan ON")

def fan_off():
    GPIO.output(FAN_PIN, GPIO.LOW)
    print("Fan OFF")

def rc_time(pin):
    count = 0
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
    time.sleep(0.1)
    GPIO.setup(pin, GPIO.IN)
    while GPIO.input(pin) == GPIO.LOW and count < MAX_COUNT:
        count += 1
    return count

# State variables
servo_open = False
last_open_time = time.time()
send_soil = True 
# ------------------ BLYNK HANDLERS ------------------

@blynk.on("V0")
def control_led1(value):
    GPIO.output(LED1, int(value[0]))
    print(f"LED1 {'ON' if int(value[0]) else 'OFF'}")

@blynk.on("V1")
def control_led2(value):
    GPIO.output(LED2, int(value[0]))
    print(f"LED2 {'ON' if int(value[0]) else 'OFF'}")

@blynk.on("V5")
def control_fan(value):
    if int(value[0]) == 1:
        fan_on()
    else:
        fan_off()

@blynk.on("V6")
def control_servo_from_blynk(value):
    global servo_open, last_open_time
    val = int(value[0])
    if val == 1 and not servo_open:
        print("Servo opened from Blynk")
        set_angle(100)
        servo_open = True
        last_open_time = time.time()
        blynk.virtual_write(6, 1)
            
@blynk.on("V8")
def control_both_leds(value):
    state = int(value[0])
    GPIO.output(LED1, state)
    GPIO.output(LED2, state)
    print(f"LED1 and LED2 {'ON' if state else 'OFF'}")
    
    blynk.virtual_write(0, state)  # V0
    blynk.virtual_write(1, state)  # V1
    
@blynk.on("connected")
def on_connected():
    print("Connected to Blynk")

# ------------------ SENSOR MONITORING ------------------

def monitor_sensors():
    smoke = GPIO.input(SMOKE_SENSOR)
    flame = GPIO.input(FLAME_SENSOR)
    print(smoke)
    
    GPIO.output(BUZZER_SMOKE, GPIO.HIGH if smoke == 0 else GPIO.LOW)
    GPIO.output(BUZZER_FLAME, GPIO.HIGH if flame == 0 else GPIO.LOW)

    blynk.virtual_write(2, smoke)
    blynk.virtual_write(3, flame)

timer.set_interval(1, monitor_sensors)

# LDR monitoring using timer callback
def monitor_ldr():
    count = rc_time(LDR_PIN)
    reading = int((count / MAX_COUNT) * MAX_READING)
    inverted_reading = MAX_READING - reading

    print(f"LDR light level (0-1023 inverted): {inverted_reading}")

    threshold = 250
    if inverted_reading < threshold:
        GPIO.output(LDR_LED1_PIN, GPIO.HIGH)
        GPIO.output(LDR_LED2_PIN, GPIO.HIGH)
    else:
        GPIO.output(LDR_LED1_PIN, GPIO.LOW)
        GPIO.output(LDR_LED2_PIN, GPIO.LOW)

timer.set_interval(1, monitor_ldr)

# ------------------ RFID SCANNER ------------------

known_uid = [39, 188, 58, 3, 162]  # Known UID
reader = MFRC522()

def rfid_scanner():
    global servo_open, last_open_time
    print("[INFO] Waiting for RFID card...")

    while True:
        (status, TagType) = reader.MFRC522_Request(reader.PICC_REQIDL)

        if status == reader.MI_OK:
            (status, uid) = reader.MFRC522_Anticoll()
            if status == reader.MI_OK:
                print(f"[INFO] Card UID: {uid}")
                if uid == known_uid and not servo_open:
                    print("? Correct UID - Opening door")
                    set_angle(100)
                    servo_open = True
                    last_open_time = time.time()
                    blynk.virtual_write(6, 1)
                else:
                    print("? Incorrect UID - Access Denied")
                    GPIO.output(BUZZER_SMOKE, GPIO.HIGH)
                    time.sleep(3)
                    GPIO.output(BUZZER_SMOKE, GPIO.LOW)
                time.sleep(2)

# Start RFID scanning in background thread
rfid_thread = threading.Thread(target=rfid_scanner, daemon=True)
rfid_thread.start()

# ------------------ MAIN LOOP ------------------

try:
    while True:
        blynk.run()
        timer.run()

        # Close servo after 5 seconds
        if servo_open and (time.time() - last_open_time >= 5):
            print("? Closing servo after 5 seconds")
            set_angle(0)
            servo_open = False
            blynk.virtual_write(6, 0)

        sleep(0.1)

except KeyboardInterrupt:
    print("\n[INFO] Exiting...")
    fan_off()
    pwm.stop()
    GPIO.cleanup()