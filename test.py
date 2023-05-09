from pydrake.all import RollPitchYaw, Quaternion
import numpy as np

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

print(2/ 4. *np.ones([1, 1, 1, 1]))#test.tolist())

