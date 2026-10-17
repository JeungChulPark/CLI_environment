# The OpenLORIS evaluator imports `from tf import transformations` (ROS 1). Christoph Gohlke's `transformations`
# package (pip) has the same functions, except that ROS tf orders quaternions [x, y, z, w] and the pip package
# orders them [w, x, y, z]. Only the four functions the evaluator calls are exposed; the two that take or return
# quaternions are reordered to the ROS convention.
import numpy as np
import transformations as _t

compose_matrix = _t.compose_matrix
rotation_from_matrix = _t.rotation_from_matrix


def quaternion_from_matrix(matrix):
    w, x, y, z = _t.quaternion_from_matrix(matrix)
    return np.array([x, y, z, w])


def euler_from_quaternion(quaternion, axes="sxyz"):
    x, y, z, w = quaternion
    return _t.euler_from_quaternion([w, x, y, z], axes)
