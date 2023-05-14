from pydrake.all import RollPitchYaw, Quaternion
import numpy as np
import math
import utils

test = np.diag(([10.0] * 6 + [1.0] * 6) * 3)

print(f'test: {test}')

