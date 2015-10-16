__author__ = 'Austin'
import cv2

# pixel_distance = location_pixel - center_pixel
# camera_c = sqrt(camera_a^2 + camera_b^2)
# angle_per_pixel = camera_view_angle / camera_c
# angle_to_object = angle_per_pixel * pixel_distance
# distance = tan(angle_to_object) * height
# if distance changes too fast or jumps around don't publish
