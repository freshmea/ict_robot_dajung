from zumi.zumi import Zumi
zumi = Zumi()

left_sensor_threshold = 100
right_sensor_threshold = 100

zumi.reset_drive()

zumi.forward_avoid_collision(speed = 20, duration=5, desired_angle = 0, left_th = left_sensor_threshold, right_th = right_sensor_threshold)
zumi.turn_right(desired_angle=90, duration = 1)
zumi.forward_avoid_collision(speed = 20, duration=5, desired_angle = -90, left_th = left_sensor_threshold, right_th = right_sensor_threshold)
zumi.turn_left(desired_angle=90, duration = 1)