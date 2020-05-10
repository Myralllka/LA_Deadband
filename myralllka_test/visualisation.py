import numpy as np
import matplotlib.pyplot as plt
import random
import matplotlib.patches as patches


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
    """
    Function check_reference. The intention is to check whether  the input
    reference is in or out from our deadband factor.
  For this, you need to project the reference into the subspace between
  target  and drone. The norm of this projection will be our deadBand factor.
  This factor will decide if we are changing the reference or not. (is it in
  deadBand region or not )

  If we decide to move the reference, hence the factor is lower then the
  threshold, you should create a orthogonal complement to the line which is
  the plane through the drone, perpendicular to the line
  This part may get quite tricky, but the main difference here that the
  projector for this plane is the same as previous, but substitute from the
  identity matrix The new reference is obtained as previous projection,
  but using the new projector + cmd_odom
    """
    subspace = np.subtract(target, uav_odometry)
    projection_matrix = np.true_divide(np.outer(subspace, subspace),
                                       np.dot(subspace, subspace))
    orthogonal_projection_matrix = np.subtract(np.identity(3),
                                               projection_matrix)
    # orthogonal_projection_matrix = np.subtract(projection_matrix, np.identity(2))
    projection = np.matmul(projection_matrix,
                           np.subtract(reference, uav_odometry))
    length = np.sqrt(np.dot(np.subtract(uav_odom, reference),
                            np.subtract(uav_odom, reference)))
    print(length)
    # print(projection)
    # print(np.sqrt(np.dot(projection, projection)))
    if length < factor:
        return (np.add(np.matmul(orthogonal_projection_matrix,
                                 np.subtract(reference, uav_odometry)),
                       uav_odometry), np.add(projection, uav_odom))
        # return np.add(projection, uav_odometry)
    else:
        return reference, np.add(projection, uav_odom)


target = np.array([10, 10, 0])
original_reference = np.array([10.5, 7.6, 0])
uav_odom = np.array([10, 1, 0])
deadBand = 4  # [m]
N = 15
DELAY = 2
flag = True
for i in range(N):
    noisex = random.choice(np.random.normal(0, 1, 100))/10
    noisey = random.choice(np.random.normal(0, 1, 100))/10
    new_ref, projection = check_reference(target, uav_odom,
                                          original_reference,
                                          deadBand)

    if np.array_equal(new_ref, original_reference) and flag:
        uav_odom = np.add(uav_odom, np.array([0, 1, 0]))
        new_ref, projection = check_reference(target, uav_odom,
                                              original_reference,
                                              deadBand)
    else:
        flag = False
        uav_odom = np.add(uav_odom, np.array([noisex, noisey, 0]))
        new_ref, projection = check_reference(target, uav_odom,
                                              original_reference,
                                              deadBand)
        original_reference = new_ref
    # react = patches.Rectangle((uav_odom[0] - deadBand, uav_odom[1] - deadBand),
    #                           deadBand * 2, deadBand * 2, edgecolor='r',
    #                           fill=False)
    circle = plt.Circle((uav_odom[0], uav_odom[1]), deadBand, color='r',
                        fill=False)
    fig, ax = plt.subplots()
    ax.set_aspect(1)
    # ax.add_artist(react)
    ax.add_artist(circle)

    plt.plot(uav_odom[0], uav_odom[1], 'bo', label='uav odom')
    plt.plot(target[0], target[1], 'ro', label='target')

    plt.plot(original_reference[0], original_reference[1], 'bx',
             label='original reference')

    plt.plot(new_ref[0], new_ref[1], 'go', label='new reference')
    plt.plot(projection[0], projection[1], 'yo', label='projection')

    plt.legend(loc="upper left")
    plt.axis('equal')
    plt.draw()
    plt.pause(DELAY)
    if i == N - 1:
        try:
            while True:
                pass
        except KeyboardInterrupt:
            break
    plt.close()
print("done")
