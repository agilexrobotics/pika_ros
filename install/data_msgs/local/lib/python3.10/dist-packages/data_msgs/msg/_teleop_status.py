# generated from rosidl_generator_py/resource/_idl.py.em
# with input from data_msgs:msg/TeleopStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TeleopStatus(type):
    """Metaclass of message 'TeleopStatus'."""

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
                'data_msgs.msg.TeleopStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__teleop_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__teleop_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__teleop_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__teleop_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__teleop_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TeleopStatus(metaclass=Metaclass_TeleopStatus):
    """Message class 'TeleopStatus'."""

    __slots__ = [
        '_topics',
        '_fail',
        '_quit',
    ]

    _fields_and_field_types = {
        'topics': 'sequence<string>',
        'fail': 'boolean',
        'quit': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.topics = kwargs.get('topics', [])
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
