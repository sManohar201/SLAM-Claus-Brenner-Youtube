# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# Claus Brenner, 26 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""

    # store the previous offset
    new_offset = a.offset + b.offset
    # initialize an array for the posterior distribution of size: size(pos)+2
    new_values = [0] * (len(a.values)+2)
    # print(new_values)
    # outter for loop, loops over the prior distribution (size: len(move))
    for i in range(len(a.values)):
    # inner for loop, loops over the individual conditional probability (size: len(prior))
        for j in range(len(b.values)):
            new_values[i+j] += a.values[i]*b.values[j]
        
    return Distribution(new_offset, new_values)  # Replace this by your own result.


if __name__ == '__main__':
    arena = (0,1000)

    # Move 3 times by 20.
    moves = [20] * 50 

    # Start with a known position: probability 1.0 at position 10.
    position = Distribution.unit_pulse(10)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Now move and plot.
    for m in moves:
        move_distribution = Distribution.triangle(m, 2)
        position = convolve(position, move_distribution)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             linestyle='steps')

    ylim(0.0, 1.1)
    show()
