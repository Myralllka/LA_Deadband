import numpy as np
import matplotlib.pyplot as plt
from numpy import linalg as la


#   For this, you need to
#   1) project the reference into the subspace between target and drone.
#         - The norm of this projection will be our deadBand factor.
#   This factor will decide if we are changing the reference or not. (is it in
#   deadBand region or not )
#   If we are updating the reference, we should project the reference to the
#   drone's plane. The goal is not move the drone ( aka vehicle ) closer or
#   further from the object is the factor is not big enough, we should only
#   move it in it's plane.


def check_reference(target: np.array, uav_odom: np.array, reference, factor):
    """
    Dead Band control
    :return: the updated or not updated reference
    """
    # use UAV pos as origin
    sub_sp: np.array = np.subtract(target, uav_odom)
    sub_sp_t: np.array = np.transpose(sub_sp)

    # p = a at / at a
    projector: np.array = np.true_divide(np.outer(sub_sp, sub_sp_t),
                                         np.dot(sub_sp_t, sub_sp))

    fake_ref: np.array = np.subtract(reference, uav_odom)
    err = la.norm(np.subtract(np.matmul(projector, fake_ref), fake_ref))
    return reference if err < factor else target


# the observed target, we want to keep track of it
target = np.array([10, 10, 0])

# uav_odom - our vehicle position
uav_odom = np.array([10, 1, 0])

# position where we want to put the vehicle
original_reference = np.array([10, 8, 0])

# the deadBand factor that will help to decide to change the reference or not.
deadBand = 3  # [m]

# this one that should be executed by our vehicle.
new_ref = check_reference(target, uav_odom, original_reference, deadBand)

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
plt.show()
