# generated from rosidl_generator_py/resource/_idl.py.em
# with input from data_msgs:msg/CaptureStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'count_in_seconds'
# Member 'frequencies'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CaptureStatus(type):
    """Metaclass of message 'CaptureStatus'."""

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
                'data_msgs.msg.CaptureStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__capture_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__capture_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__capture_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__capture_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__capture_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CaptureStatus(metaclass=Metaclass_CaptureStatus):
    """Message class 'CaptureStatus'."""

    __slots__ = [
        '_topics',
        '_count_in_seconds',
        '_frequencies',
        '_fail',
        '_quit',
    ]

    _fields_and_field_types = {
        'topics': 'sequence<string>',
        'count_in_seconds': 'sequence<int32>',
        'frequencies': 'sequence<float>',
        'fail': 'boolean',
        'quit': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.topics = kwargs.get('topics', [])
        self.count_in_seconds = array.array('i', kwargs.get('count_in_seconds', []))
        self.frequencies = array.array('f', kwargs.get('frequencies', []))
        self.fail = kwargs.get('fail', bool())
        self.quit = kwargs.get('quit', bool())

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
        if self.topics != other.topics:
            return False
        if self.count_in_seconds != other.count_in_seconds:
            return False
        if self.frequencies != other.frequencies:
            return False
        if self.fail != other.fail:
            return False
        if self.quit != other.quit:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def topics(self):
        """Message field 'topics'."""
        return self._topics

    @topics.setter
    def topics(self, value):
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
                "The 'topics' field must be a set or sequence and each value of type 'str'"
        self._topics = value

    @builtins.property
    def count_in_seconds(self):
        """Message field 'count_in_seconds'."""
        return self._count_in_seconds

    @count_in_seconds.setter
    def count_in_seconds(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'count_in_seconds' array.array() must have the type code of 'i'"
            self._count_in_seconds = value
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
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'count_in_seconds' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._count_in_seconds = array.array('i', value)

    @builtins.property
    def frequencies(self):
        """Message field 'frequencies'."""
        return self._frequencies

    @frequencies.setter
    def frequencies(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'frequencies' array.array() must have the type code of 'f'"
            self._frequencies = value
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
                "The 'frequencies' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._frequencies = array.array('f', value)

    @builtins.property
    def fail(self):
        """Message field 'fail'."""
        return self._fail

    @fail.setter
    def fail(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fail' field must be of type 'bool'"
        self._fail = value

    @builtins.property  # noqa: A003
    def quit(self):  # noqa: A003
        """Message field 'quit'."""
        return self._quit

    @quit.setter  # noqa: A003
    def quit(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'quit' field must be of type 'bool'"
        self._quit = value
