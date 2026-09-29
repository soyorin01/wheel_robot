# generated from rosidl_generator_py/resource/_idl.py.em
# with input from motor_temperature_pkg:msg/MotorTemperatures.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'temperature_c'
# Member 'overheated_index'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorTemperatures(type):
    """Metaclass of message 'MotorTemperatures'."""

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
            module = import_type_support('motor_temperature_pkg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'motor_temperature_pkg.msg.MotorTemperatures')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_temperatures
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_temperatures
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_temperatures
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_temperatures
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_temperatures

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


class MotorTemperatures(metaclass=Metaclass_MotorTemperatures):
    """Message class 'MotorTemperatures'."""

    __slots__ = [
        '_header',
        '_name',
        '_temperature_c',
        '_warning_threshold_c',
        '_overheated_index',
        '_overheated',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'name': 'sequence<string>',
        'temperature_c': 'sequence<float>',
        'warning_threshold_c': 'float',
        'overheated_index': 'sequence<uint8>',
        'overheated': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint8')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.name = kwargs.get('name', [])
        self.temperature_c = array.array('f', kwargs.get('temperature_c', []))
        self.warning_threshold_c = kwargs.get('warning_threshold_c', float())
        self.overheated_index = array.array('B', kwargs.get('overheated_index', []))
        self.overheated = kwargs.get('overheated', bool())

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
        if self.name != other.name:
            return False
        if self.temperature_c != other.temperature_c:
            return False
        if self.warning_threshold_c != other.warning_threshold_c:
            return False
        if self.overheated_index != other.overheated_index:
            return False
        if self.overheated != other.overheated:
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
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'name' field must be a set or sequence and each value of type 'str'"
        self._name = value

    @builtins.property
    def temperature_c(self):
        """Message field 'temperature_c'."""
        return self._temperature_c

    @temperature_c.setter
    def temperature_c(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'temperature_c' array.array() must have the type code of 'f'"
            self._temperature_c = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'temperature_c' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._temperature_c = array.array('f', value)

    @builtins.property
    def warning_threshold_c(self):
        """Message field 'warning_threshold_c'."""
        return self._warning_threshold_c

    @warning_threshold_c.setter
    def warning_threshold_c(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'warning_threshold_c' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'warning_threshold_c' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._warning_threshold_c = value

    @builtins.property
    def overheated_index(self):
        """Message field 'overheated_index'."""
        return self._overheated_index

    @overheated_index.setter
    def overheated_index(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'B', \
                "The 'overheated_index' array.array() must have the type code of 'B'"
            self._overheated_index = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'overheated_index' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 255]"
        self._overheated_index = array.array('B', value)

    @builtins.property
    def overheated(self):
        """Message field 'overheated'."""
        return self._overheated

    @overheated.setter
    def overheated(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'overheated' field must be of type 'bool'"
        self._overheated = value
