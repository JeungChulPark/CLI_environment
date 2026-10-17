# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hdl_graph_slam:msg/ScanMatchingOdometryDebug.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ScanMatchingOdometryDebug(type):
    """Metaclass of message 'ScanMatchingOdometryDebug'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('hdl_graph_slam')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hdl_graph_slam.msg.ScanMatchingOdometryDebug')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__scan_matching_odometry_debug
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__scan_matching_odometry_debug
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__scan_matching_odometry_debug
            cls._TYPE_SUPPORT = module.type_support_msg__msg__scan_matching_odometry_debug
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__scan_matching_odometry_debug

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ScanMatchingOdometryDebug(metaclass=Metaclass_ScanMatchingOdometryDebug):
    """Message class 'ScanMatchingOdometryDebug'."""

    __slots__ = [
        '_header',
        '_input_point_count',
        '_downsampled_point_count',
        '_registration_time_ms',
        '_callback_time_ms',
        '_odom_count',
        '_registration_triggered',
        '_has_converged',
        '_matching_error',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'input_point_count': 'uint32',
        'downsampled_point_count': 'uint32',
        'registration_time_ms': 'float',
        'callback_time_ms': 'float',
        'odom_count': 'uint64',
        'registration_triggered': 'boolean',
        'has_converged': 'boolean',
        'matching_error': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.input_point_count = kwargs.get('input_point_count', int())
        self.downsampled_point_count = kwargs.get('downsampled_point_count', int())
        self.registration_time_ms = kwargs.get('registration_time_ms', float())
        self.callback_time_ms = kwargs.get('callback_time_ms', float())
        self.odom_count = kwargs.get('odom_count', int())
        self.registration_triggered = kwargs.get('registration_triggered', bool())
        self.has_converged = kwargs.get('has_converged', bool())
        self.matching_error = kwargs.get('matching_error', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.input_point_count != other.input_point_count:
            return False
        if self.downsampled_point_count != other.downsampled_point_count:
            return False
        if self.registration_time_ms != other.registration_time_ms:
            return False
        if self.callback_time_ms != other.callback_time_ms:
            return False
        if self.odom_count != other.odom_count:
            return False
        if self.registration_triggered != other.registration_triggered:
            return False
        if self.has_converged != other.has_converged:
            return False
        if self.matching_error != other.matching_error:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def input_point_count(self):
        """Message field 'input_point_count'."""
        return self._input_point_count

    @input_point_count.setter
    def input_point_count(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'input_point_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'input_point_count' field must be an unsigned integer in [0, 4294967295]"
        self._input_point_count = value

    @builtins.property
    def downsampled_point_count(self):
        """Message field 'downsampled_point_count'."""
        return self._downsampled_point_count

    @downsampled_point_count.setter
    def downsampled_point_count(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'downsampled_point_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'downsampled_point_count' field must be an unsigned integer in [0, 4294967295]"
        self._downsampled_point_count = value

    @builtins.property
    def registration_time_ms(self):
        """Message field 'registration_time_ms'."""
        return self._registration_time_ms

    @registration_time_ms.setter
    def registration_time_ms(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'registration_time_ms' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'registration_time_ms' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._registration_time_ms = value

    @builtins.property
    def callback_time_ms(self):
        """Message field 'callback_time_ms'."""
        return self._callback_time_ms

    @callback_time_ms.setter
    def callback_time_ms(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'callback_time_ms' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'callback_time_ms' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._callback_time_ms = value

    @builtins.property
    def odom_count(self):
        """Message field 'odom_count'."""
        return self._odom_count

    @odom_count.setter
    def odom_count(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'odom_count' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'odom_count' field must be an unsigned integer in [0, 18446744073709551615]"
        self._odom_count = value

    @builtins.property
    def registration_triggered(self):
        """Message field 'registration_triggered'."""
        return self._registration_triggered

    @registration_triggered.setter
    def registration_triggered(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'registration_triggered' field must be of type 'bool'"
        self._registration_triggered = value

    @builtins.property
    def has_converged(self):
        """Message field 'has_converged'."""
        return self._has_converged

    @has_converged.setter
    def has_converged(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'has_converged' field must be of type 'bool'"
        self._has_converged = value

    @builtins.property
    def matching_error(self):
        """Message field 'matching_error'."""
        return self._matching_error

    @matching_error.setter
    def matching_error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'matching_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'matching_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._matching_error = value
