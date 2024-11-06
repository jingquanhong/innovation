# Untitled - By: dell - Mon Oct 21 2024

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

red_threshold=(30,100,15,127,127);
green_threshold=(54，93，-77，-44，30，57);
sensor.set_auto_gain(False);

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
