import RPi.GPIO as GPIO
import subprocess
import os

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

main_script = os.path.join(os.path.join(os.path.dirname(__file__)), "wro.py")
stop_script = os.path.join(os.path.join(os.path.dirname(__file__)), "stop.py")

while True:
    if GPIO.input(10) == GPIO.HIGH:
        subprocess.run(["python3", main_script])
        break
    else:
        subprocess.run(["python3", stop_script])
        break