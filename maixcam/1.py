from maix import gpio, pinmap, time, sys, err

pin_name = "B25" if sys.device_id() == "maixcam2" else "B3"
gpio_name = "GPIOB25" if sys.device_id() == "maixcam2" else "GPIOB3"

err.check_raise(pinmap.set_pin_function(pin_name, gpio_name), "set pin failed")
led = gpio.GPIO(gpio_name, gpio.Mode.OUT)
led.value()

while 1:
    led.value(1)
