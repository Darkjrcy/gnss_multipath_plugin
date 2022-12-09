# generated from rosidl_generator_py/resource/_idl.py.em
# with input from multipath_sim:msg/MultipathOffset.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'range_offset'
# Member 'sats_blocked'
import array  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MultipathOffset(type):
    """Metaclass of message 'MultipathOffset'."""

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
            module = import_type_support('multipath_sim')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'multipath_sim.msg.MultipathOffset')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__multipath_offset
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__multipath_offset
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__multipath_offset
            cls._TYPE_SUPPORT = module.type_support_msg__msg__multipath_offset
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__multipath_offset

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


class MultipathOffset(metaclass=Metaclass_MultipathOffset):
    """Message class 'MultipathOffset'."""

    __slots__ = [
        '_header',
        '_range_offset',
        '_sats_blocked',
        '_num_vis_sat',
        '_num_block_sat',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'range_offset': 'sequence<float>',
        'sats_blocked': 'sequence<int32>',
        'num_vis_sat': 'int32',
        'num_block_sat': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.range_offset = array.array('f', kwargs.get('range_offset', []))
        self.sats_blocked = array.array('i', kwargs.get('sats_blocked', []))
        self.num_vis_sat = kwargs.get('num_vis_sat', int())
        self.num_block_sat = kwargs.get('num_block_sat', int())

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
        if self.range_offset != other.range_offset:
            return False
        if self.sats_blocked != other.sats_blocked:
            return False
        if self.num_vis_sat != other.num_vis_sat:
            return False
        if self.num_block_sat != other.num_block_sat:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
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

    @property
    def range_offset(self):
        """Message field 'range_offset'."""
        return self._range_offset

    @range_offset.setter
    def range_offset(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'range_offset' array.array() must have the type code of 'f'"
            self._range_offset = value
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
                 True), \
                "The 'range_offset' field must be a set or sequence and each value of type 'float'"
        self._range_offset = array.array('f', value)

    @property
    def sats_blocked(self):
        """Message field 'sats_blocked'."""
        return self._sats_blocked

    @sats_blocked.setter
    def sats_blocked(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'sats_blocked' array.array() must have the type code of 'i'"
            self._sats_blocked = value
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
                "The 'sats_blocked' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._sats_blocked = array.array('i', value)

    @property
    def num_vis_sat(self):
        """Message field 'num_vis_sat'."""
        return self._num_vis_sat

    @num_vis_sat.setter
    def num_vis_sat(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_vis_sat' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num_vis_sat' field must be an integer in [-2147483648, 2147483647]"
        self._num_vis_sat = value

    @property
    def num_block_sat(self):
        """Message field 'num_block_sat'."""
        return self._num_block_sat

    @num_block_sat.setter
    def num_block_sat(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_block_sat' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num_block_sat' field must be an integer in [-2147483648, 2147483647]"
        self._num_block_sat = value
