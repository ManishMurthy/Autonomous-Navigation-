# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vox_nav_msgs:action/NavigateThroughGPSPoses.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_NavigateThroughGPSPoses_Goal(type):
    """Metaclass of message 'NavigateThroughGPSPoses_Goal'."""

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
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_Goal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_through_gps_poses__goal
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_through_gps_poses__goal
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_through_gps_poses__goal
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_through_gps_poses__goal
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_through_gps_poses__goal

            from sensor_msgs.msg import NavSatFix
            if NavSatFix.__class__._TYPE_SUPPORT is None:
                NavSatFix.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateThroughGPSPoses_Goal(metaclass=Metaclass_NavigateThroughGPSPoses_Goal):
    """Message class 'NavigateThroughGPSPoses_Goal'."""

    __slots__ = [
        '_gps_poses',
    ]

    _fields_and_field_types = {
        'gps_poses': 'sequence<sensor_msgs/NavSatFix>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'NavSatFix')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.gps_poses = kwargs.get('gps_poses', [])

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
        if self.gps_poses != other.gps_poses:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def gps_poses(self):
        """Message field 'gps_poses'."""
        return self._gps_poses

    @gps_poses.setter
    def gps_poses(self, value):
        if __debug__:
            from sensor_msgs.msg import NavSatFix
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
                 all(isinstance(v, NavSatFix) for v in value) and
                 True), \
                "The 'gps_poses' field must be a set or sequence and each value of type 'NavSatFix'"
        self._gps_poses = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateThroughGPSPoses_Result(type):
    """Metaclass of message 'NavigateThroughGPSPoses_Result'."""

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
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_Result')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_through_gps_poses__result
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_through_gps_poses__result
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_through_gps_poses__result
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_through_gps_poses__result
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_through_gps_poses__result

            from std_msgs.msg import Empty
            if Empty.__class__._TYPE_SUPPORT is None:
                Empty.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateThroughGPSPoses_Result(metaclass=Metaclass_NavigateThroughGPSPoses_Result):
    """Message class 'NavigateThroughGPSPoses_Result'."""

    __slots__ = [
        '_result',
    ]

    _fields_and_field_types = {
        'result': 'std_msgs/Empty',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Empty'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Empty
        self.result = kwargs.get('result', Empty())

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
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from std_msgs.msg import Empty
            assert \
                isinstance(value, Empty), \
                "The 'result' field must be a sub message of type 'Empty'"
        self._result = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateThroughGPSPoses_Feedback(type):
    """Metaclass of message 'NavigateThroughGPSPoses_Feedback'."""

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
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_Feedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_through_gps_poses__feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_through_gps_poses__feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_through_gps_poses__feedback
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_through_gps_poses__feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_through_gps_poses__feedback

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateThroughGPSPoses_Feedback(metaclass=Metaclass_NavigateThroughGPSPoses_Feedback):
    """Message class 'NavigateThroughGPSPoses_Feedback'."""

    __slots__ = [
        '_current_pose',
        '_navigation_time',
        '_distance_remaining',
    ]

    _fields_and_field_types = {
        'current_pose': 'geometry_msgs/PoseStamped',
        'navigation_time': 'builtin_interfaces/Duration',
        'distance_remaining': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import PoseStamped
        self.current_pose = kwargs.get('current_pose', PoseStamped())
        from builtin_interfaces.msg import Duration
        self.navigation_time = kwargs.get('navigation_time', Duration())
        self.distance_remaining = kwargs.get('distance_remaining', float())

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
        if self.current_pose != other.current_pose:
            return False
        if self.navigation_time != other.navigation_time:
            return False
        if self.distance_remaining != other.distance_remaining:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def current_pose(self):
        """Message field 'current_pose'."""
        return self._current_pose

    @current_pose.setter
    def current_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'current_pose' field must be a sub message of type 'PoseStamped'"
        self._current_pose = value

    @builtins.property
    def navigation_time(self):
        """Message field 'navigation_time'."""
        return self._navigation_time

    @navigation_time.setter
    def navigation_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'navigation_time' field must be a sub message of type 'Duration'"
        self._navigation_time = value

    @builtins.property
    def distance_remaining(self):
        """Message field 'distance_remaining'."""
        return self._distance_remaining

    @distance_remaining.setter
    def distance_remaining(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_remaining' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance_remaining' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance_remaining = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateThroughGPSPoses_SendGoal_Request(type):
    """Metaclass of message 'NavigateThroughGPSPoses_SendGoal_Request'."""

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
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_SendGoal_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_through_gps_poses__send_goal__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_through_gps_poses__send_goal__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_through_gps_poses__send_goal__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_through_gps_poses__send_goal__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_through_gps_poses__send_goal__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

            from vox_nav_msgs.action import NavigateThroughGPSPoses
            if NavigateThroughGPSPoses.Goal.__class__._TYPE_SUPPORT is None:
                NavigateThroughGPSPoses.Goal.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateThroughGPSPoses_SendGoal_Request(metaclass=Metaclass_NavigateThroughGPSPoses_SendGoal_Request):
    """Message class 'NavigateThroughGPSPoses_SendGoal_Request'."""

    __slots__ = [
        '_goal_id',
        '_goal',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'vox_nav_msgs/NavigateThroughGPSPoses_Goal',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['vox_nav_msgs', 'action'], 'NavigateThroughGPSPoses_Goal'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_Goal
        self.goal = kwargs.get('goal', NavigateThroughGPSPoses_Goal())

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
        if self.goal_id != other.goal_id:
            return False
        if self.goal != other.goal:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def goal(self):
        """Message field 'goal'."""
        return self._goal

    @goal.setter
    def goal(self, value):
        if __debug__:
            from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_Goal
            assert \
                isinstance(value, NavigateThroughGPSPoses_Goal), \
                "The 'goal' field must be a sub message of type 'NavigateThroughGPSPoses_Goal'"
        self._goal = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateThroughGPSPoses_SendGoal_Response(type):
    """Metaclass of message 'NavigateThroughGPSPoses_SendGoal_Response'."""

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
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_SendGoal_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_through_gps_poses__send_goal__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_through_gps_poses__send_goal__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_through_gps_poses__send_goal__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_through_gps_poses__send_goal__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_through_gps_poses__send_goal__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateThroughGPSPoses_SendGoal_Response(metaclass=Metaclass_NavigateThroughGPSPoses_SendGoal_Response):
    """Message class 'NavigateThroughGPSPoses_SendGoal_Response'."""

    __slots__ = [
        '_accepted',
        '_stamp',
    ]

    _fields_and_field_types = {
        'accepted': 'boolean',
        'stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.accepted = kwargs.get('accepted', bool())
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())

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
        if self.accepted != other.accepted:
            return False
        if self.stamp != other.stamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def accepted(self):
        """Message field 'accepted'."""
        return self._accepted

    @accepted.setter
    def accepted(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'accepted' field must be of type 'bool'"
        self._accepted = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value


class Metaclass_NavigateThroughGPSPoses_SendGoal(type):
    """Metaclass of service 'NavigateThroughGPSPoses_SendGoal'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_SendGoal')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__navigate_through_gps_poses__send_goal

            from vox_nav_msgs.action import _navigate_through_gps_poses
            if _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_SendGoal_Request._TYPE_SUPPORT is None:
                _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_SendGoal_Request.__import_type_support__()
            if _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_SendGoal_Response._TYPE_SUPPORT is None:
                _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_SendGoal_Response.__import_type_support__()


class NavigateThroughGPSPoses_SendGoal(metaclass=Metaclass_NavigateThroughGPSPoses_SendGoal):
    from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_SendGoal_Request as Request
    from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_SendGoal_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateThroughGPSPoses_GetResult_Request(type):
    """Metaclass of message 'NavigateThroughGPSPoses_GetResult_Request'."""

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
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_GetResult_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_through_gps_poses__get_result__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_through_gps_poses__get_result__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_through_gps_poses__get_result__request
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_through_gps_poses__get_result__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_through_gps_poses__get_result__request

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateThroughGPSPoses_GetResult_Request(metaclass=Metaclass_NavigateThroughGPSPoses_GetResult_Request):
    """Message class 'NavigateThroughGPSPoses_GetResult_Request'."""

    __slots__ = [
        '_goal_id',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())

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
        if self.goal_id != other.goal_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateThroughGPSPoses_GetResult_Response(type):
    """Metaclass of message 'NavigateThroughGPSPoses_GetResult_Response'."""

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
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_GetResult_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_through_gps_poses__get_result__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_through_gps_poses__get_result__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_through_gps_poses__get_result__response
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_through_gps_poses__get_result__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_through_gps_poses__get_result__response

            from vox_nav_msgs.action import NavigateThroughGPSPoses
            if NavigateThroughGPSPoses.Result.__class__._TYPE_SUPPORT is None:
                NavigateThroughGPSPoses.Result.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateThroughGPSPoses_GetResult_Response(metaclass=Metaclass_NavigateThroughGPSPoses_GetResult_Response):
    """Message class 'NavigateThroughGPSPoses_GetResult_Response'."""

    __slots__ = [
        '_status',
        '_result',
    ]

    _fields_and_field_types = {
        'status': 'int8',
        'result': 'vox_nav_msgs/NavigateThroughGPSPoses_Result',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['vox_nav_msgs', 'action'], 'NavigateThroughGPSPoses_Result'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())
        from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_Result
        self.result = kwargs.get('result', NavigateThroughGPSPoses_Result())

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
        if self.status != other.status:
            return False
        if self.result != other.result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
            assert value >= -128 and value < 128, \
                "The 'status' field must be an integer in [-128, 127]"
        self._status = value

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_Result
            assert \
                isinstance(value, NavigateThroughGPSPoses_Result), \
                "The 'result' field must be a sub message of type 'NavigateThroughGPSPoses_Result'"
        self._result = value


class Metaclass_NavigateThroughGPSPoses_GetResult(type):
    """Metaclass of service 'NavigateThroughGPSPoses_GetResult'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_GetResult')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__action__navigate_through_gps_poses__get_result

            from vox_nav_msgs.action import _navigate_through_gps_poses
            if _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_GetResult_Request._TYPE_SUPPORT is None:
                _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_GetResult_Request.__import_type_support__()
            if _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_GetResult_Response._TYPE_SUPPORT is None:
                _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_GetResult_Response.__import_type_support__()


class NavigateThroughGPSPoses_GetResult(metaclass=Metaclass_NavigateThroughGPSPoses_GetResult):
    from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_GetResult_Request as Request
    from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_GetResult_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_NavigateThroughGPSPoses_FeedbackMessage(type):
    """Metaclass of message 'NavigateThroughGPSPoses_FeedbackMessage'."""

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
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses_FeedbackMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__action__navigate_through_gps_poses__feedback_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__action__navigate_through_gps_poses__feedback_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__action__navigate_through_gps_poses__feedback_message
            cls._TYPE_SUPPORT = module.type_support_msg__action__navigate_through_gps_poses__feedback_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__action__navigate_through_gps_poses__feedback_message

            from unique_identifier_msgs.msg import UUID
            if UUID.__class__._TYPE_SUPPORT is None:
                UUID.__class__.__import_type_support__()

            from vox_nav_msgs.action import NavigateThroughGPSPoses
            if NavigateThroughGPSPoses.Feedback.__class__._TYPE_SUPPORT is None:
                NavigateThroughGPSPoses.Feedback.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class NavigateThroughGPSPoses_FeedbackMessage(metaclass=Metaclass_NavigateThroughGPSPoses_FeedbackMessage):
    """Message class 'NavigateThroughGPSPoses_FeedbackMessage'."""

    __slots__ = [
        '_goal_id',
        '_feedback',
    ]

    _fields_and_field_types = {
        'goal_id': 'unique_identifier_msgs/UUID',
        'feedback': 'vox_nav_msgs/NavigateThroughGPSPoses_Feedback',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['unique_identifier_msgs', 'msg'], 'UUID'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['vox_nav_msgs', 'action'], 'NavigateThroughGPSPoses_Feedback'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from unique_identifier_msgs.msg import UUID
        self.goal_id = kwargs.get('goal_id', UUID())
        from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_Feedback
        self.feedback = kwargs.get('feedback', NavigateThroughGPSPoses_Feedback())

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
        if self.goal_id != other.goal_id:
            return False
        if self.feedback != other.feedback:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_id(self):
        """Message field 'goal_id'."""
        return self._goal_id

    @goal_id.setter
    def goal_id(self, value):
        if __debug__:
            from unique_identifier_msgs.msg import UUID
            assert \
                isinstance(value, UUID), \
                "The 'goal_id' field must be a sub message of type 'UUID'"
        self._goal_id = value

    @builtins.property
    def feedback(self):
        """Message field 'feedback'."""
        return self._feedback

    @feedback.setter
    def feedback(self, value):
        if __debug__:
            from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_Feedback
            assert \
                isinstance(value, NavigateThroughGPSPoses_Feedback), \
                "The 'feedback' field must be a sub message of type 'NavigateThroughGPSPoses_Feedback'"
        self._feedback = value


class Metaclass_NavigateThroughGPSPoses(type):
    """Metaclass of action 'NavigateThroughGPSPoses'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('vox_nav_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vox_nav_msgs.action.NavigateThroughGPSPoses')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_action__action__navigate_through_gps_poses

            from action_msgs.msg import _goal_status_array
            if _goal_status_array.Metaclass_GoalStatusArray._TYPE_SUPPORT is None:
                _goal_status_array.Metaclass_GoalStatusArray.__import_type_support__()
            from action_msgs.srv import _cancel_goal
            if _cancel_goal.Metaclass_CancelGoal._TYPE_SUPPORT is None:
                _cancel_goal.Metaclass_CancelGoal.__import_type_support__()

            from vox_nav_msgs.action import _navigate_through_gps_poses
            if _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_SendGoal._TYPE_SUPPORT is None:
                _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_SendGoal.__import_type_support__()
            if _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_GetResult._TYPE_SUPPORT is None:
                _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_GetResult.__import_type_support__()
            if _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_FeedbackMessage._TYPE_SUPPORT is None:
                _navigate_through_gps_poses.Metaclass_NavigateThroughGPSPoses_FeedbackMessage.__import_type_support__()


class NavigateThroughGPSPoses(metaclass=Metaclass_NavigateThroughGPSPoses):

    # The goal message defined in the action definition.
    from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_Goal as Goal
    # The result message defined in the action definition.
    from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_Result as Result
    # The feedback message defined in the action definition.
    from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_Feedback as Feedback

    class Impl:

        # The send_goal service using a wrapped version of the goal message as a request.
        from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_SendGoal as SendGoalService
        # The get_result service using a wrapped version of the result message as a response.
        from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_GetResult as GetResultService
        # The feedback message with generic fields which wraps the feedback message.
        from vox_nav_msgs.action._navigate_through_gps_poses import NavigateThroughGPSPoses_FeedbackMessage as FeedbackMessage

        # The generic service to cancel a goal.
        from action_msgs.srv._cancel_goal import CancelGoal as CancelGoalService
        # The generic message for get the status of a goal.
        from action_msgs.msg._goal_status_array import GoalStatusArray as GoalStatusMessage

    def __init__(self):
        raise NotImplementedError('Action classes can not be instantiated')
