# generated from rosidl_generator_py/resource/_idl.py.em
# with input from wheel_robot:msg/StepDetection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StepDetection(type):
    """Metaclass of message 'StepDetection'."""

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
            module = import_type_support('wheel_robot')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'wheel_robot.msg.StepDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__step_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__step_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__step_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__step_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__step_detection

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


class StepDetection(metaclass=Metaclass_StepDetection):
    """Message class 'StepDetection'."""

    __slots__ = [
        '_header',
        '_detected',
        '_distance',
        '_height',
        '_confidence',
        '_pose_valid',
        '_edge_angle_rad',
        '_lateral_offset_m',
        '_confirm_frames',
        '_required_confirm_frames',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'detected': 'boolean',
        'distance': 'float',
        'height': 'float',
        'confidence': 'float',
        'pose_valid': 'boolean',
        'edge_angle_rad': 'float',
        'lateral_offset_m': 'float',
        'confirm_frames': 'uint32',
        'required_confirm_frames': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.detected = kwargs.get('detected', bool())
        self.distance = kwargs.get('distance', float())
        self.height = kwargs.get('height', float())
        self.confidence = kwargs.get('confidence', float())
        self.pose_valid = kwargs.get('pose_valid', bool())
        self.edge_angle_rad = kwargs.get('edge_angle_rad', float())
        self.lateral_offset_m = kwargs.get('lateral_offset_m', float())
        self.confirm_frames = kwargs.get('confirm_frames', int())
        self.required_confirm_frames = kwargs.get('required_confirm_frames', int())

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
        if self.detected != other.detected:
            return False
        if self.distance != other.distance:
            return False
        if self.height != other.height:
            return False
        if self.confidence != other.confidence:
            return False
        if self.pose_valid != other.pose_valid:
            return False
        if self.edge_angle_rad != other.edge_angle_rad:
            return False
        if self.lateral_offset_m != other.lateral_offset_m:
            return False
        if self.confirm_frames != other.confirm_frames:
            return False
        if self.required_confirm_frames != other.required_confirm_frames:
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
    def detected(self):
        """Message field 'detected'."""
        return self._detected

    @detected.setter
    def detected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'detected' field must be of type 'bool'"
        self._detected = value

    @builtins.property
    def distance(self):
        """Message field 'distance'."""
        return self._distance

    @distance.setter
    def distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance = value

    @builtins.property
    def height(self):
        """Message field 'height'."""
        return self._height

    @height.setter
    def height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'height' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'height' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._height = value

    @builtins.property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence = value

    @builtins.property
    def pose_valid(self):
        """Message field 'pose_valid'."""
        return self._pose_valid

    @pose_valid.setter
    def pose_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pose_valid' field must be of type 'bool'"
        self._pose_valid = value

    @builtins.property
    def edge_angle_rad(self):
        """Message field 'edge_angle_rad'."""
        return self._edge_angle_rad

    @edge_angle_rad.setter
    def edge_angle_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'edge_angle_rad' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'edge_angle_rad' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._edge_angle_rad = value

    @builtins.property
    def lateral_offset_m(self):
        """Message field 'lateral_offset_m'."""
        return self._lateral_offset_m

    @lateral_offset_m.setter
    def lateral_offset_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lateral_offset_m' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'lateral_offset_m' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._lateral_offset_m = value

    @builtins.property
    def confirm_frames(self):
        """Message field 'confirm_frames'."""
        return self._confirm_frames

    @confirm_frames.setter
    def confirm_frames(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'confirm_frames' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'confirm_frames' field must be an unsigned integer in [0, 4294967295]"
        self._confirm_frames = value

    @builtins.property
    def required_confirm_frames(self):
        """Message field 'required_confirm_frames'."""
        return self._required_confirm_frames

    @required_confirm_frames.setter
    def required_confirm_frames(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'required_confirm_frames' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'required_confirm_frames' field must be an unsigned integer in [0, 4294967295]"
        self._required_confirm_frames = value
