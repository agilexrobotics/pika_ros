# generated from rosidl_generator_py/resource/_idl.py.em
# with input from data_msgs:srv/CaptureService.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CaptureService_Request(type):
    """Metaclass of message 'CaptureService_Request'."""

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
                'data_msgs.srv.CaptureService_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__capture_service__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__capture_service__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__capture_service__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__capture_service__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__capture_service__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CaptureService_Request(metaclass=Metaclass_CaptureService_Request):
    """Message class 'CaptureService_Request'."""

    __slots__ = [
        '_start',
        '_end',
        '_episode_index',
        '_dataset_dir',
        '_instructions',
    ]

    _fields_and_field_types = {
        'start': 'boolean',
        'end': 'boolean',
        'episode_index': 'int32',
        'dataset_dir': 'string',
        'instructions': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.start = kwargs.get('start', bool())
        self.end = kwargs.get('end', bool())
        self.episode_index = kwargs.get('episode_index', int())
        self.dataset_dir = kwargs.get('dataset_dir', str())
        self.instructions = kwargs.get('instructions', str())

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
        if self.start != other.start:
            return False
        if self.end != other.end:
            return False
        if self.episode_index != other.episode_index:
            return False
        if self.dataset_dir != other.dataset_dir:
            return False
        if self.instructions != other.instructions:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def start(self):
        """Message field 'start'."""
        return self._start

    @start.setter
    def start(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'start' field must be of type 'bool'"
        self._start = value

    @builtins.property
    def end(self):
        """Message field 'end'."""
        return self._end

    @end.setter
    def end(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'end' field must be of type 'bool'"
        self._end = value

    @builtins.property
    def episode_index(self):
        """Message field 'episode_index'."""
        return self._episode_index

    @episode_index.setter
    def episode_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'episode_index' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'episode_index' field must be an integer in [-2147483648, 2147483647]"
        self._episode_index = value

    @builtins.property
    def dataset_dir(self):
        """Message field 'dataset_dir'."""
        return self._dataset_dir

    @dataset_dir.setter
    def dataset_dir(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'dataset_dir' field must be of type 'str'"
        self._dataset_dir = value

    @builtins.property
    def instructions(self):
        """Message field 'instructions'."""
        return self._instructions

    @instructions.setter
    def instructions(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'instructions' field must be of type 'str'"
        self._instructions = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CaptureService_Response(type):
    """Metaclass of message 'CaptureService_Response'."""

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
                'data_msgs.srv.CaptureService_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__capture_service__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__capture_service__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__capture_service__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__capture_service__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__capture_service__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CaptureService_Response(metaclass=Metaclass_CaptureService_Response):
    """Message class 'CaptureService_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_CaptureService(type):
    """Metaclass of service 'CaptureService'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('data_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'data_msgs.srv.CaptureService')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__capture_service

            from data_msgs.srv import _capture_service
            if _capture_service.Metaclass_CaptureService_Request._TYPE_SUPPORT is None:
                _capture_service.Metaclass_CaptureService_Request.__import_type_support__()
            if _capture_service.Metaclass_CaptureService_Response._TYPE_SUPPORT is None:
                _capture_service.Metaclass_CaptureService_Response.__import_type_support__()


class CaptureService(metaclass=Metaclass_CaptureService):
    from data_msgs.srv._capture_service import CaptureService_Request as Request
    from data_msgs.srv._capture_service import CaptureService_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
