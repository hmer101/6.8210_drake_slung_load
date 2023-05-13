from pydrake.all import RollPitchYaw, Quaternion
import numpy as np
import math
import utils

# Define the roll, pitch, and yaw angles (in radians)
roll = 0.0
pitch = 0.0
yaw = 0.0

# Create a RollPitchYaw object from the RPY angles
rpy = RollPitchYaw(roll, pitch, yaw)

# Convert the RollPitchYaw object to a Quaternion
quaternion = Quaternion(rpy.ToQuaternion())

# Print the quaternion components (qw, qx, qy, qz)
#print(quaternion.wxyz())


list1 = [1,2,3]
list2 = [4,5,6]
test = np.array(list1)
test2 = np.ones(5)

x0, u0 = utils.get_stable_state(3, 1, 2.0, 0.5, 0.1, math.pi/4, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

print(f'x0_len: {len(x0)}') #test.tolist())
print(f'x0: {x0}')
print(f'u0: {u0}')

