# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hdl_graph_slam:msg/PrefilteringDebug.idl
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


class Metaclass_PrefilteringDebug(type):
    """Metaclass of message 'PrefilteringDebug'."""

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
                'hdl_graph_slam.msg.PrefilteringDebug')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__prefiltering_debug
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__prefiltering_debug
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__prefiltering_debug
            cls._TYPE_SUPPORT = module.type_support_msg__msg__prefiltering_debug
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__prefiltering_debug

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


class PrefilteringDebug(metaclass=Metaclass_PrefilteringDebug):
    """Message class 'PrefilteringDebug'."""

    __slots__ = [
        '_header',
        '_input_point_count',
        '_transformed_point_count',
        '_distance_filtered_point_count',
        '_downsampled_point_count',
        '_output_point_count',
        '_callback_time_ms',
        '_output_count',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'input_point_count': 'uint32',
        'transformed_point_count': 'uint32',
        'distance_filtered_point_count': 'uint32',
        'downsampled_point_count': 'uint32',
        'output_point_count': 'uint32',
        'callback_time_ms': 'float',
        'output_count': 'uint64',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
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
        self.transformed_point_count = kwargs.get('transformed_point_count', int())
        self.distance_filtered_point_count = kwargs.get('distance_filtered_point_count', int())
        self.downsampled_point_count = kwargs.get('downsampled_point_count', int())
        self.output_point_count = kwargs.get('output_point_count', int())
        self.callback_time_ms = kwargs.get('callback_time_ms', float())
        self.output_count = kwargs.get('output_count', int())

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
        if self.transformed_point_count != other.transformed_point_count:
            return False
        if self.distance_filtered_point_count != other.distance_filtered_point_count:
            return False
        if self.downsampled_point_count != other.downsampled_point_count:
            return False
        if self.output_point_count != other.output_point_count:
            return False
        if self.callback_time_ms != other.callback_time_ms:
            return False
        if self.output_count != other.output_count:
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
    def transformed_point_count(self):
        """Message field 'transformed_point_count'."""
        return self._transformed_point_count

    @transformed_point_count.setter
    def transformed_point_count(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'transformed_point_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'transformed_point_count' field must be an unsigned integer in [0, 4294967295]"
        self._transformed_point_count = value

    @builtins.property
    def distance_filtered_point_count(self):
        """Message field 'distance_filtered_point_count'."""
        return self._distance_filtered_point_count

    @distance_filtered_point_count.setter
    def distance_filtered_point_count(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'distance_filtered_point_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'distance_filtered_point_count' field must be an unsigned integer in [0, 4294967295]"
        self._distance_filtered_point_count = value

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
    def output_point_count(self):
        """Message field 'output_point_count'."""
        return self._output_point_count

    @output_point_count.setter
    def output_point_count(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'output_point_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'output_point_count' field must be an unsigned integer in [0, 4294967295]"
        self._output_point_count = value

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
    def output_count(self):
        """Message field 'output_count'."""
        return self._output_count

    @output_count.setter
    def output_count(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'output_count' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'output_count' field must be an unsigned integer in [0, 18446744073709551615]"
        self._output_count = value
