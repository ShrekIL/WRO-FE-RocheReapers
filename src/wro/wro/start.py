import lgpio
import subprocess
import os

BUTTON_GPIO = 10

h = lgpio.gpiochip_open(0)
lgpio.set_mode(h, BUTTON_GPIO, lgpio.INPUT)
lgpio.set_pull_up_down(h, BUTTON_GPIO, lgpio.PUD_DOWN)

main_script = os.path.join(os.path.join(os.path.dirname(__file__)), "wro.py")
stop_script = os.path.join(os.path.join(os.path.dirname(__file__)), "stop.py")

try:
    while True:
        if lgpio.gpio_read(h, BUTTON_GPIO) == 1:
            subprocess.run(["python3", main_script])
            break
        else:
            subprocess.run(["python3", stop_script])
            break
        #time.sleep(0.1)
finally:
    lgpio.gpiochip_close(h)
