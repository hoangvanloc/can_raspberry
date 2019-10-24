import RPi.GPIO as GPIO
LED2_PIN = 10
LEDST_PIN = 11
def LED_Init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED2_PIN, GPIO.OUT)
    GPIO.setup(LEDST_PIN, GPIO.OUT)
def BSP_LED2_ON():
    GPIO.output(LED2_PIN, GPIO.HIGH)
def BSP_LED2_OFF():
    GPIO.output(LED2_PIN, GPIO.LOW)
def BSP_LED2_TOGGLE():
    if GPIO.input(LED2_PIN) == 1:
        BSP_LED2_OFF()
    else:
        BSP_LED2_ON()
def BSP_STLED_ON():
    GPIO.output(LEDST_PIN, GPIO.HIGH)
def BSP_STLED_OFF():
    GPIO.output(LEDST_PIN, GPIO.LOW)
def BSP_STLED_Toggle():
    if GPIO.input(LEDST_PIN) == 1:
        BSP_STLED_OFF()
    else:
        BSP_STLED_ON()