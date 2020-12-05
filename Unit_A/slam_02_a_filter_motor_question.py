# Implement the first move model for the Lego robot.
# 02_a_filter_motor
# Claus Brenner, 31 OCT 2012
from math import sin, cos, pi
from pylab import *
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width):
    # unpack left and right ticks from the motor ticks variable
    ticks_l, ticks_r = motor_ticks
    # convert the ticks to mm
    ticks_left = ticks_l*ticks_to_mm 
    ticks_right = ticks_r*ticks_to_mm
    # upack old pose to seperate variables 
    old_x, old_y, old_theta = old_pose 
    # Find out if there is a turn at all.
    if ticks_l == ticks_r:
        # No turn. Just drive straight.
        x = old_x + ticks_left*cos(old_theta)
        y = old_y + ticks_right*sin(old_theta)
        theta = old_theta
        return (x, y, theta)
    else:
        # define the alpha angle
        alpha = (ticks_right - ticks_left)/robot_width
        # define the radius of instantaneous center
        radius_ICC = ticks_left/alpha
        # calculate instantaneous center
        cx = old_x - (radius_ICC+robot_width/2)*sin(old_theta)
        cy = old_y + (radius_ICC+robot_width/2)*cos(old_theta)
        # define current heading of the robot
        theta = (old_theta+alpha)%(2*pi)
        # define current position of the robot
        x = cx + (radius_ICC+robot_width/2)*sin(theta)
        y = cy + (radius_ICC+robot_width/2)*(-cos(theta))
        return (x, y, theta)

if __name__ == '__main__':
    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 173.0

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Start at origin (0,0), looking along x axis (alpha = 0).
    pose = (0.0, 0.0, 0.0)

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width)
        filtered.append(pose)

    # Draw result.
    for pose in filtered:
        print pose
        plot([p[0] for p in filtered], [p[1] for p in filtered], 'bo')
    show()
