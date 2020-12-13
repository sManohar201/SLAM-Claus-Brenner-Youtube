# The particle filter, prediciton and correction.
#
# slam_08_b_particle_correction.
# Claus Brenner, 04.01.2013
from lego_robot import *
from slam_e_library import get_cylinders_from_scan, assign_cylinders
from math import sin, cos, pi, atan2, sqrt
import random
import numpy as np
from scipy.stats import norm as normal_dist


class ParticleFilter:
    def __init__(self, initial_particles,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The particles.
        self.particles = initial_particles

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

    # State transition. This is exactly the same method as in the Kalman filter.
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

        return (g1, g2, g3)

    def predict(self, control):
        """The prediction step of the particle filter."""
        l, r = control
        # Compute left and right variance.
        # alpha_1 is self.control_motion_factor.
        # alpha_2 is self.control_turn_factor.
        sigma2_l = (self.control_motion_factor*l)**2 + (self.control_turn_factor*(l-r))**2
        sigma2_r = (self.control_motion_factor*r)**2 + (self.control_turn_factor*(l-r))**2
        # Then, do a loop over all self.particles and construct a new
        # list of particles.
        new_particles = []
        for particle in self.particles:
            # For sampling, use random.gauss(mu, sigma). (Note sigma in this call
            # is the standard deviation, not the variance.)   
            right_p = random.gauss(r, sqrt(sigma2_r))
            left_p = random.gauss(l, sqrt(sigma2_l))
            new_particles.append(self.g(particle, (left_p, right_p), self.robot_width))
        
        # In the end, assign the new list of particles to self.particles.
        self.particles = new_particles

    # Measurement. This is exactly the same method as in the Kalman filter.
    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           corresponding (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return (r, alpha)

    def probability_of_measurement(self, measurement, predicted_measurement):
        """Given a measurement and a predicted measurement, computes
           probability."""
        # Compute difference in distance and bearing angle.
        # Important: make sure the angle difference works correctly and does
        # not return values offset by 2 pi or - 2 pi.
        # You may use the following Gaussian PDF function:
        # scipy.stats.norm.pdf(x, mu, sigma). With the import in the header,
        # this is normal_dist.pdf(x, mu, sigma).
        # Note that the two parameters sigma_d and sigma_alpha discussed
        # in the lecture are self.measurement_distance_stddev and
        # self.measurement_angle_stddev.
        sigma_d = self.measurement_distance_stddev
        sigma_alpha = self.measurement_angle_stddev

        z = np.array(measurement)
        pred_z = np.array(predicted_measurement)
        z[1] = atan2(sin(z[1]), cos(z[1]))
        pred_z[1] = atan2(sin(pred_z[1]), cos(pred_z[1]))

        delta = z - pred_z
        delta[1] = atan2(sin(delta[1]), cos(delta[1]))

        P_d = normal_dist.pdf(delta[0], 0, sigma_d)
        P_alpha = normal_dist.pdf(delta[1], 0, sigma_alpha)

        return P_d * P_alpha

    def compute_weights(self, cylinders, landmarks):
        """Computes one weight for each particle, returns list of weights."""
        weights = []
        for p in self.particles:
            # Get list of tuples:
            # [ ((range_0, bearing_0), (landmark_x, landmark_y)), ... ]
            # print(p)
            assignment = assign_cylinders(cylinders, p,
                self.scanner_displacement, landmarks)

            wt = 1.0
            for z, landmark in assignment:
                # Get measurement for given landmark
                z_pred = self.h(p, landmark, self.scanner_displacement)
                wt *= self.probability_of_measurement(z, z_pred)
            weights.append(wt)
        # total = sum(weights)
        # return [w / total for w in weights]
        return weights

    def resample(self, weights):
        """Return a list of particles which have been resampled, proportional
           to the given weights."""
        # You may implement the 'resampling wheel' algorithm
        # described in the lecture.
        new_particles = []
        max_weight = max(weights)
        len_weight = len(weights)
        ind = random.randint(0, len_weight-1)
        offset = 0
        for i in range(len_weight):
            offset += random.random() * 2 * max_weight
            while offset >= weights[ind]:
                offset -= weights[ind]
                ind = (ind + 1)%len_weight
            new_particles.append(self.particles[ind]) 
            
        return new_particles
    
    def correct(self, cylinders, landmarks):
        """The correction step of the particle filter."""
        # First compute all weights.
        weights = self.compute_weights(cylinders, landmarks)
        # Then resample, based on the weight array.
        self.particles = self.resample(weights)

    def print_particles(self, file_desc):
        """Prints particles to given file_desc output."""
        if not self.particles:
            return
        print >> file_desc, "PA",
        for p in self.particles:
            print >> file_desc, "%.0f %.0f %.3f" % p,
        print >> file_desc


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Generate initial particles. Each particle is (x, y, theta).
    number_of_particles = 50
    measured_state = (1850.0, 1897.0, 213.0 / 180.0 * pi)
    standard_deviations = (100.0, 100.0, 10.0 / 180.0 * pi)
    initial_particles = []
    for i in xrange(number_of_particles):
        initial_particles.append(tuple([
            random.gauss(measured_state[j], standard_deviations[j])
            for j in xrange(3)]))

    # Setup filter.
    pf = ParticleFilter(initial_particles,
                        robot_width, scanner_displacement,
                        control_motion_factor, control_turn_factor,
                        measurement_distance_stddev,
                        measurement_angle_stddev)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]
    # Loop over all motor tick records.
    # This is the particle filter loop, with prediction and correction.
    f = open("particle_filter_corrected.txt", "w")
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        print("Iteration" + " : " + str(i))
        control = map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i])
        pf.predict(control)

        # Correction.
        cylinders = get_cylinders_from_scan(logfile.scan_data[i], depth_jump,
            minimum_valid_distance, cylinder_offset)
        # print(len(cylinders))
        # print(len(reference_cylinders))
        pf.correct(cylinders, reference_cylinders)

        # Output particles.
        pf.print_particles(f)

    f.close()
