import dvrk
import rclpy
import time

rclpy.init()

time.sleep(0.5)
m = dvrk.mtm("MTMR")

m.body.servo_cf([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
time.sleep(0.5)
m.use_gravity_compensation(True)