import lgpio

BUTTON_GPIO = 15

h = lgpio.gpiochip_open(4)

lgpio.gpio_claim_input(h, BUTTON_GPIO, lFlags=lgpio.SET_PULL_UP)

ready = False

def is_ready():
    global ready
    if ready:
        return True
    
    if lgpio.gpio_read(h, BUTTON_GPIO) == 0:
        ready = True
        return True
    
    return False