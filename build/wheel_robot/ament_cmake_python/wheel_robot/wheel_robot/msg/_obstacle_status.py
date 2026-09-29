# generated from rosidl_generator_py/resource/_idl.py.em
# with input from wheel_robot:msg/ObstacleStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ObstacleStatus(type):
    """Metaclass of message 'ObstacleStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'UNKNOWN': 0,
        'CLEAR': 1,
        'CAUTION': 2,
        'STOP': 3,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('wheel_robot')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'wheel_robot.msg.ObstacleStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__obstacle_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__obstacle_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__obstacle_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__obstacle_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__obstacle_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'UNKNOWN': cls.__constants['UNKNOWN'],
            'CLEAR': cls.__constants['CLEAR'],
            'CAUTION': cls.__constants['CAUTION'],
            'STOP': cls.__constants['STOP'],
        }

    @property
    def UNKNOWN(self):
        """Message constant 'UNKNOWN'."""
        return Metaclass_ObstacleStatus.__constants['UNKNOWN']

    @property
    def CLEAR(self):
        """Message constant 'CLEAR'."""
        return Metaclass_ObstacleStatus.__constants['CLEAR']

    @property
    def CAUTION(self):
        """Message constant 'CAUTION'."""
        return Metaclass_ObstacleStatus.__constants['CAUTION']

    @property
    def STOP(self):
        """Message constant 'STOP'."""
        return Metaclass_ObstacleStatus.__constants['STOP']


class ObstacleStatus(metaclass=Metaclass_ObstacleStatus):
    """
    Message class 'ObstacleStatus'.

    Constants:
      UNKNOWN
      CLEAR
      CAUTION
      STOP
    """

    __slots__ = [
        '_header',
        '_status',
        '_left_status',
        '_center_status',
        '_right_status',
        '_left_distance',
        '_center_distance',
        '_right_distance',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'status': 'uint8',
        'left_status': 'uint8',
        'center_status': 'uint8',
        'right_status': 'uint8',
        'left_distance': 'float',
        'center_distance': 'float',
        'right_distance': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.status = kwargs.get('status', int())
        self.left_status = kwargs.get('left_status', int())
        self.center_status = kwargs.get('center_status', int())
        self.right_status = kwargs.get('right_status', int())
        self.left_distance = kwargs.get('left_distance', float())
        self.center_distance = kwargs.get('center_distance', float())
        self.right_distance = kwargs.get('right_distance', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.status != other.status:
            return False
        if self.left_status != other.left_status:
            return False
        if self.center_status != other.center_status:
            return False
        if self.right_status != other.right_status:
            return False
        if self.left_distance != other.left_distance:
            return False
        if self.center_distance != other.center_distance:
            return False
        if self.right_distance != other.right_distance:
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
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value

    @builtins.property
    def left_status(self):
        """Message field 'left_status'."""
        return self._left_status

    @left_status.setter
    def left_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'left_status' field must be an unsigned integer in [0, 255]"
        self._left_status = value

    @builtins.property
    def center_status(self):
        """Message field 'center_status'."""
        return self._center_status

    @center_status.setter
    def center_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'center_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'center_status' field must be an unsigned integer in [0, 255]"
        self._center_status = value

    @builtins.property
    def right_status(self):
        """Message field 'right_status'."""
        return self._right_status

    @right_status.setter
    def right_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'right_status' field must be an unsigned integer in [0, 255]"
        self._right_status = value

    @builtins.property
    def left_distance(self):
        """Message field 'left_distance'."""
        return self._left_distance

    @left_distance.setter
    def left_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'left_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._left_distance = value

    @builtins.property
    def center_distance(self):
        """Message field 'center_distance'."""
        return self._center_distance

    @center_distance.setter
    def center_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'center_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'center_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._center_distance = value

    @builtins.property
    def right_distance(self):
        """Message field 'right_distance'."""
        return self._right_distance

    @right_distance.setter
    def right_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'right_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._right_distance = value
