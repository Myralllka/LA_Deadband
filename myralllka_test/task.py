import numpy as np
import matplotlib.pyplot as plt


# Task
# Usually when you do an operation with a non-stationary vehicle ( like UAV,
# copter ) like following, or idling near an sensor observed object to avoid
# and prohibit any oscillation of the vehicle due to false-positive detection,
# sensor noise, etc ( lot's of problem occurs when you go out from your
# simulation, lab to the real world conditions ).
# There are multiple solution to this problem:
#   ~ Filtering the data - this will smooth out the movement and get rid of
#   the noise, and will make sure that if there are drastic changes in position
#   of the observed object that it will gradually change the position.
#   ~ DeadBand control - even with filtered position you might have a slight
#   movement of the object smth like 10-12cm and if you have predefined
#   observing distance for your vehicle exactly 1 meter, you will oscillate
#   around that point which may cause multiple tragic outcomes ( smth like
#   your colleague hunting you in the desert because you destroyed the drone ).
# We will go with the deadBand solution since it's much more simple to
# implement. ( but who knows ).

# # Dead Band - https://en.wikipedia.org/wiki/Deadband
# So, what should you do. We have 4 variables:
#    - target - the observed target, we want to keep track of it
#    - uav_odom - our vehicle position
#    - original_refence - position where we want to put the vehicle, in theory
#    it's output of the calculation from the detection that was obtained from
#    the sensor.
#    - deadBand - is the deadBand factor that will help to decide to change the
#    reference or not.
#    - new_ref - this one that should be executed by our vehicle.
# Function check_reference. The intention is to check whether  the input
# refence is in or out from our deadband factor.
#   For this, you need to
#   TODO: project the reference into the subspace between target and drone.
#    The norm of this projection will be our deadBand factor.
#   TODO: This factor will decide if we are changing the reference or not. (
#    is it in deadBand region or not )
#   If we are updating the reference, we should project the reference to the
#   drone's plane. The goal is not move the drone ( aka vehicle ) closer or
#   further from the object is the factor is not big enough, we should only
#   move it in it's plane.

def check_reference(target, uav_odometry, reference, factor):
    subspace = np.subtract(target, uav_odometry)
    projection_matrix = np.true_divide(np.outer(subspace, subspace),
                                       np.dot(subspace, subspace))
    res = np.matmul(projection_matrix, np.subtract(reference,
                                                   uav_odometry))
    res = np.add(res, uav_odometry)
    # if we wont to update on target
    reference = target if (np.dot(reference, res) > factor) else reference
    # or update to a projection
    # reference = res if (np.dot(reference, res) > factor) else reference
    return reference  # return the updated or not updated reference


target = np.array([10, 10, 0])

uav_odom = np.array([5, 1, 0])

original_reference = np.array([12, 7, 0])

deadBand = 1  # [m]

new_ref = check_reference(target, uav_odom, original_reference, deadBand)
# plt.xticks(np.arange(0, 2, step=0.2))
# plt.yticks(np.arange(0, 2, step=0.2))yscale(
# plt.yscale('linear')
# plt.xscale('linear')
# plot the target red
plt.plot(target[0], target[1], 'ro', label='target')
# plot the uav blue
plt.plot(uav_odom[0], uav_odom[1], 'bo', label='uav odom')

# plot the old reference
plt.plot(original_reference[0], original_reference[1], 'bx',
         label='original reference')

# plot the new reference
plt.plot(new_ref[0], new_ref[1], 'go', label='new reference')
plt.legend(loc="upper left")
plt.axis('equal')
plt.show()
