from time import time
from threading import Thread
import time
from pymycobot.ultraArm import ultraArm
# from ultraArm.megaAiKit import megaAikit
from megaAiKit import megaAikit

aikit = megaAikit("COM9")
ua = ultraArm("COM6", 115200)

down_x = -6.91
down_y = 252.1
first_point_anges = [91.57, 7.6, 0.16]
leave_to_default_angles = [91.57, 0, 0]
slide_rail_up_angles = [38, 10, -3]
slide_rail_down_angles = [38, 15, 23]

"""
    Initialize the suction point.
"""


def initial_point(speed, millisecond):
    ua.set_angles(first_point_anges, speed)
    time.sleep(millisecond)


"""
    Suction object.
"""


def pickup(down_z, speed, millisecond):
    ua.set_coords([down_x, down_y, down_z], speed)
    time.sleep(millisecond)

    """open suction pump"""
    ua.set_gpio_state(0)
    time.sleep(millisecond)

    ua.set_angles(leave_to_default_angles, speed)
    time.sleep(millisecond)


"""
    Move to position above slide rail.
"""


def move_to_slide_rail(robo_speed, millisecond):
    ua.set_angles(slide_rail_up_angles, robo_speed)
    time.sleep(millisecond)

    ua.set_angles(slide_rail_down_angles, robo_speed)
    time.sleep(millisecond)

    """shut down the suction pump"""
    ua.set_gpio_state(1)
    time.sleep(millisecond)

    """open slide rail, speed 60"""
    aikit.write_steps_by_switch(1, 60)
    time.sleep(2.2)

    ua.set_angles(slide_rail_up_angles, robo_speed)
    time.sleep(millisecond)

    """close rail"""
    aikit.write_steps_by_switch(0, 0)
    time.sleep(1)


"""
    Detects the valid specified range for sensor ranging.
"""


def detect_tof_distance(dist, min_range, max_range):
    if dist >= min_range and dist <= max_range:
        return True
    else:
        return False


"""
    The task of getting the distance.
"""


class ThreadTof(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.running = True
        self.dist = None

    def run(self):
        while self.running:
            if (not self.running):
                self.dist = None
                break
            else:
                dist = aikit.get_tof_distance()
                if (None != dist):
                    self.dist = dist
                    # print(self.dist)
                time.sleep(0.5)

    def get_tof_distance(self):
        return self.dist

    def set_flag(self, flag):
        self.running = flag


default_down_z_postion = 126.47
down_z_offset = 40
lower_distance_limit = 245
upper_distance_limit = 373
# absorbed_distance = 0

"""
    Motion Planning Main Program.
"""


def motion_plan(robo_speed, millisecond):
    tof_thread = ThreadTof()
    tof_thread.daemon = True
    tof_thread.start()

    ua.go_zero()

    # down_z = default_down_z_postion
    # for i in range(4):
    #     if 0 == i:
    #         down_z -= 20
    #     else:
    #         down_z -= 40
    #     initial_point(robo_speed, millisecond)
    #     pickup(down_z, robo_speed, millisecond)
    #     move_to_slide_rail(robo_speed, slide_rail_speed, millisecond)

    # global absorbed_distance
    while True:
        down_z = default_down_z_postion
        count = -1
        is_detect = False
        curr_detect_dist = tof_thread.get_tof_distance()
        print("Current detected distance:", curr_detect_dist)

        if lower_distance_limit <= curr_detect_dist and upper_distance_limit >= curr_detect_dist:
            to_distance = lower_distance_limit + down_z_offset  # 285
            # if detect_tof_distance(curr_detect_dist, lower_distance_limit, to_distance):
            if detect_tof_distance(curr_detect_dist, 245, 285):
                # absorbed_distance += to_distance + 1
                is_detect = True
                count = 1
            # if detect_tof_distance(curr_detect_dist, absorbed_distance, absorbed_distance + down_z_offset + 9):
            if detect_tof_distance(curr_detect_dist, 286, 335):  # 286 335
                # absorbed_distance += absorbed_distance + down_z_offset + 10
                is_detect = True
                count = 2
            # if detect_tof_distance(curr_detect_dist, absorbed_distance, absorbed_distance + down_z_offset - 26):
            if detect_tof_distance(curr_detect_dist, 336, 356):  # 336 356
                # absorbed_distance += absorbed_distance + down_z_offset - 25
                is_detect = True
                count = 3
            # if detect_tof_distance(curr_detect_dist, absorbed_distance, upper_distance_limit):
            if detect_tof_distance(curr_detect_dist, 357, 373):
                is_detect = True
                count = 4
        else:
            print("Detect out of range!")

        if is_detect:
            if -1 <= count and 4 >= count:
                if -1 == count:
                    """position of the default initial point"""
                    down_z = default_down_z_postion
                elif 1 == count:
                    """starting point position of the first object"""
                    down_z -= down_z_offset / 2
                else:
                    """calculating the dynamic position of an object"""
                    down_z -= (count * down_z_offset) - (down_z_offset / 2)
                print('count:', count)
                print("Current Z axis point:", down_z)
                initial_point(robo_speed, millisecond)
                pickup(down_z, robo_speed, millisecond)
                move_to_slide_rail(robo_speed, millisecond)
                is_detect = False


# motion_plan(95, 1)

# while True:
#     print(aikit.get_tof_distance())
#     time.sleep(0.5)

aikit.write_steps_by_switch(1, 50)
time.sleep(5)
aikit.write_steps_by_switch(0, 50)