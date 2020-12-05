# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *

# This funcition takes the body frame coordinates and converts it into lidar frame coordinates
# (body_pose, scan_displacement) and return lidar_pose
def frame_conversion(body_pose, scan_displacement):
    y = body_pose[1]
    y += scan_displacement
    return (body_pose[0], y, body_pose[2]) 

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):
    # unpack the motor ticks
    ticks_l, ticks_r = motor_ticks
    ticks_left = ticks_to_mm*ticks_l
    ticks_right = ticks_to_mm*ticks_r
    # unpack the old pose data 
    old_x, old_y, old_theta = old_pose
    # account for laser displacement
    old_y -= scanner_displacement
    # Find out if there is a turn at all.
    if ticks_r == ticks_l: 
        x = old_x + ticks_left*cos(old_theta) 
        y = old_y + ticks_right*sin(old_theta)
        theta = old_theta
        return frame_conversion((x, y, theta), scanner_displacement)

    else:
        alpha = (ticks_right - ticks_left)/robot_width
        radius_ICC = ticks_left / alpha
        inst_x = old_x - (radius_ICC+robot_width/2)*sin(old_theta)
        inst_y = old_y + (radius_ICC+robot_width/2)*cos(old_theta)
        theta = (old_theta+alpha) % (2*pi)
        x = inst_x + (radius_ICC+robot_width/2)*sin(theta)
        y = inst_y - (radius_ICC+robot_width/2)*cos(theta)
        return frame_conversion((x, y, theta), scanner_displacement)

if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 173.0

    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        print >> f, "F %f %f %f" % pose
    f.close()
