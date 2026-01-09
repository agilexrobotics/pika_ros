# generated from rosidl_generator_py/resource/_idl.py.em
# with input from data_msgs:msg/Gripper.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Gripper(type):
    """Metaclass of message 'Gripper'."""

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
            module = import_type_support('data_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'data_msgs.msg.Gripper')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gripper
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gripper
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gripper
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gripper
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gripper

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


class Gripper(metaclass=Metaclass_Gripper):
    """Message class 'Gripper'."""

    __slots__ = [
        '_header',
        '_angle',
        '_distance',
        '_effort',
        '_velocity',
        '_enable',
        '_set_zero',
        '_error',
        '_voltage',
        '_driver_temp',
        '_motor_temp',
        '_bus_current',
        '_status',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'angle': 'double',
        'distance': 'double',
        'effort': 'double',
        'velocity': 'double',
        'enable': 'boolean',
        'set_zero': 'boolean',
        'error': 'boolean',
        'voltage': 'double',
        'driver_temp': 'double',
        'motor_temp': 'double',
        'bus_current': 'double',
        'status': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.angle = kwargs.get('angle', float())
        self.distance = kwargs.get('distance', float())
        self.effort = kwargs.get('effort', float())
        self.velocity = kwargs.get('velocity', float())
        self.enable = kwargs.get('enable', bool())
        self.set_zero = kwargs.get('set_zero', bool())
        self.error = kwargs.get('error', bool())
        self.voltage = kwargs.get('voltage', float())
        self.driver_temp = kwargs.get('driver_temp', float())
        self.motor_temp = kwargs.get('motor_temp', float())
        self.bus_current = kwargs.get('bus_current', float())
        self.status = kwargs.get('status', str())

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
        if self.angle != other.angle:
            return False
        if self.distance != other.distance:
            return False
        if self.effort != other.effort:
            return False
        if self.velocity != other.velocity:
            return False
        if self.enable != other.enable:
            return False
        if self.set_zero != other.set_zero:
            return False
        if self.error != other.error:
            return False
        if self.voltage != other.voltage:
            return False
        if self.driver_temp != other.driver_temp:
            return False
        if self.motor_temp != other.motor_temp:
            return False
        if self.bus_current != other.bus_current:
            return False
        if self.status != other.status:
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
    def angle(self):
        """Message field 'angle'."""
        return self._angle

    @angle.setter
    def angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'angle' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._angle = value

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
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'distance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._distance = value

    @builtins.property
    def effort(self):
        """Message field 'effort'."""
        return self._effort

    @effort.setter
    def effort(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'effort' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'effort' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._effort = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._velocity = value

    @builtins.property
    def enable(self):
        """Message field 'enable'."""
        return self._enable

    @enable.setter
    def enable(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'enable' field must be of type 'bool'"
        self._enable = value

    @builtins.property
    def set_zero(self):
        """Message field 'set_zero'."""
        return self._set_zero

    @set_zero.setter
    def set_zero(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'set_zero' field must be of type 'bool'"
        self._set_zero = value

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'error' field must be of type 'bool'"
        self._error = value

    @builtins.property
    def voltage(self):
        """Message field 'voltage'."""
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._voltage = value

    @builtins.property
    def driver_temp(self):
        """Message field 'driver_temp'."""
        return self._driver_temp

    @driver_temp.setter
    def driver_temp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'driver_temp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'driver_temp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._driver_temp = value

    @builtins.property
    def motor_temp(self):
        """Message field 'motor_temp'."""
        return self._motor_temp

    @motor_temp.setter
    def motor_temp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor_temp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'motor_temp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._motor_temp = value

    @builtins.property
    def bus_current(self):
        """Message field 'bus_current'."""
        return self._bus_current

    @bus_current.setter
    def bus_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bus_current' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'bus_current' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._bus_current = value

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status' field must be of type 'str'"
        self._status = value
