# This adds the derivative of g, this time with respect to the control
# (left and right motor movement).
#
# slam_07_c_control_derivative
# Claus Brenner, 11.12.2012
from lego_robot import *
from math import sin, cos, pi
from numpy import *

class ExtendedKalmanFilter:

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])

    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)
        ele13 = -1/w
        ele23 = 1/w
        if r != l:
            # This is for the case l != r.
            # Note g has 3 components and control has 2, so the result
            # will be a 3x2 (rows x columns) matrix.
            alpha = (r-l)/w
            theta_p = theta+alpha
            ele11 = ((w*r)/(r-l)**2)*(sin(theta_p)-sin(theta)) - ((r+l)/(2*(r-l)))*cos(theta_p)
            ele12 = ((w*r)/(r-l)**2)*(-cos(theta_p)+cos(theta)) - ((r+l)/(2*(r-l)))*sin(theta_p)
            ele21 = ((-w*l)/(r-l)**2)*(sin(theta_p)-sin(theta)) + ((r+l)/(2*(r-l)))*cos(theta_p)
            ele22 = ((-w*l)/(r-l)**2)*(-cos(theta_p)+cos(theta)) + ((r+l)/(2*(r-l)))*sin(theta_p)

        else:
            # This is for the special case l == r.
            ele11 = 0.5 * (cos(theta) + (l/w)*sin(theta)) 
            ele12 = 0.5 * (sin(theta) - (l/w)*cos(theta))
            ele21 = 0.5 * (-(l/w)*sin(theta) + cos(theta))
            ele22 = 0.5 * ((l/w)*cos(theta) + sin(theta))

        return array([[ele11, ele21], [ele12, ele22], [ele13, ele23]])


if __name__ == '__main__':
    # If the partial derivative with respect to l and r (the control)
    # are correct, then the numerical derivative and the analytical
    # derivative should be the same.

    # Set some variables. Try other variables as well.
    # In particular, you should check cases with l == r and l != r.
    x = 10.0
    y = 20.0
    theta = 35. / 180. * pi
    state = array([x, y, theta])
    l = 50.0
    r = 54.32
    control = array([l, r])
    w = 150.0

    # Compute derivative numerically.
    print "Numeric differentiation dl, dr"
    delta = 1e-7
    control_l = array([l + delta, r])
    control_r = array([l, r + delta])
    dg_dl = (ExtendedKalmanFilter.g(state, control_l, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dr = (ExtendedKalmanFilter.g(state, control_r, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dcontrol_numeric = column_stack([dg_dl, dg_dr])
    print dg_dcontrol_numeric

    # Use the above code to compute the derivative analytically.
    print "Analytic differentiation dl, dr:"
    dg_dcontrol_analytic = ExtendedKalmanFilter.dg_dcontrol(state, control, w)
    print dg_dcontrol_analytic

    # The difference should be close to zero (depending on the setting of
    # delta, above).
    print "Difference:"
    print dg_dcontrol_numeric - dg_dcontrol_analytic
    print "Seems correct:", allclose(dg_dcontrol_numeric, dg_dcontrol_analytic)
