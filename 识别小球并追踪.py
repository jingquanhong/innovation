import sensor, image, time
from pyb import Pin

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# 定义颜色阈值
blue_threshold = (0, 100, -9, 3, -32, -14)

# 初始化引脚
p1 = Pin('P1', Pin.OUT_PP)
p0 = Pin('P0', Pin.OUT_PP)
p2 = Pin('P2', Pin.OUT_PP)

# 设置蓝色物块的最小面积，当达到这个面积时跳出循环
min_area_to_break = 10000  # 这个值可以根据实际情况调整

while True:
    img = sensor.snapshot()
    blobs = img.find_blobs([blue_threshold], pixels_threshold=10, area_threshold=10, merge=True)

    if blobs:
        # 找到面积最大的物块
        largest_blob = max(blobs, key=lambda b: b.pixels())

        # 绘制矩形和十字
        img.draw_rectangle(largest_blob.rect(), color=(255, 0, 0))
        img.draw_cross(largest_blob.cx(), largest_blob.cy(), color=(255, 0, 0))

        x, y = largest_blob.cx(), largest_blob.cy()
        print("x: {}, y: {}, area: {}".format(x, y, largest_blob.pixels()))

        # 根据位置控制引脚
        if x < 130:
            p1.low()
            p0.high()
        elif x > 190:
            p1.high()
            p0.low()
        else:
            p1.low()
            p0.low()

        # 检查蓝色物块的面积是否达到设定的最小值
        if largest_blob.pixels() >= min_area_to_break:
            # 给p2高电平
            p2.high()
            print("square")
            # 跳出循环
            break
    else:
        # 如果没有检测到物块，设置默认状态
        p1.high()
        p0.high()
        p2.low()
