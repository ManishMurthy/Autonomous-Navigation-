# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vox_nav_msgs:srv/GetTraversabilityMap.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetTraversabilityMap_Request(type):
    """Metaclass of message 'GetTraversabilityMap_Request'."""

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
                'vox_nav_msgs.srv.GetTraversabilityMap_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_traversability_map__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_traversability_map__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_traversability_map__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_traversability_map__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_traversability_map__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTraversabilityMap_Request(metaclass=Metaclass_GetTraversabilityMap_Request):
    """Message class 'GetTraversabilityMap_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetTraversabilityMap_Response(type):
    """Metaclass of message 'GetTraversabilityMap_Response'."""

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
                'vox_nav_msgs.srv.GetTraversabilityMap_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_traversability_map__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_traversability_map__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_traversability_map__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_traversability_map__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_traversability_map__response

            from geometry_msgs.msg import PoseArray
            if PoseArray.__class__._TYPE_SUPPORT is None:
                PoseArray.__class__.__import_type_support__()

            from octomap_msgs.msg import Octomap
            if Octomap.__class__._TYPE_SUPPORT is None:
                Octomap.__class__.__import_type_support__()

            from sensor_msgs.msg import PointCloud2
            if PointCloud2.__class__._TYPE_SUPPORT is None:
                PointCloud2.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetTraversabilityMap_Response(metaclass=Metaclass_GetTraversabilityMap_Response):
    """Message class 'GetTraversabilityMap_Response'."""

    __slots__ = [
        '_original_octomap',
        '_collision_octomap',
        '_elevated_surfel_octomap',
        '_elevated_surfel_poses',
        '_traversable_elevated_cloud',
        '_traversable_cloud',
        '_is_valid',
    ]

    _fields_and_field_types = {
        'original_octomap': 'octomap_msgs/Octomap',
        'collision_octomap': 'octomap_msgs/Octomap',
        'elevated_surfel_octomap': 'octomap_msgs/Octomap',
        'elevated_surfel_poses': 'geometry_msgs/PoseArray',
        'traversable_elevated_cloud': 'sensor_msgs/PointCloud2',
        'traversable_cloud': 'sensor_msgs/PointCloud2',
        'is_valid': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['octomap_msgs', 'msg'], 'Octomap'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['octomap_msgs', 'msg'], 'Octomap'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['octomap_msgs', 'msg'], 'Octomap'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseArray'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from octomap_msgs.msg import Octomap
        self.original_octomap = kwargs.get('original_octomap', Octomap())
        from octomap_msgs.msg import Octomap
        self.collision_octomap = kwargs.get('collision_octomap', Octomap())
        from octomap_msgs.msg import Octomap
        self.elevated_surfel_octomap = kwargs.get('elevated_surfel_octomap', Octomap())
        from geometry_msgs.msg import PoseArray
        self.elevated_surfel_poses = kwargs.get('elevated_surfel_poses', PoseArray())
        from sensor_msgs.msg import PointCloud2
        self.traversable_elevated_cloud = kwargs.get('traversable_elevated_cloud', PointCloud2())
        from sensor_msgs.msg import PointCloud2
        self.traversable_cloud = kwargs.get('traversable_cloud', PointCloud2())
        self.is_valid = kwargs.get('is_valid', bool())

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
        if self.original_octomap != other.original_octomap:
            return False
        if self.collision_octomap != other.collision_octomap:
            return False
        if self.elevated_surfel_octomap != other.elevated_surfel_octomap:
            return False
        if self.elevated_surfel_poses != other.elevated_surfel_poses:
            return False
        if self.traversable_elevated_cloud != other.traversable_elevated_cloud:
            return False
        if self.traversable_cloud != other.traversable_cloud:
            return False
        if self.is_valid != other.is_valid:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def original_octomap(self):
        """Message field 'original_octomap'."""
        return self._original_octomap

    @original_octomap.setter
    def original_octomap(self, value):
        if __debug__:
            from octomap_msgs.msg import Octomap
            assert \
                isinstance(value, Octomap), \
                "The 'original_octomap' field must be a sub message of type 'Octomap'"
        self._original_octomap = value

    @builtins.property
    def collision_octomap(self):
        """Message field 'collision_octomap'."""
        return self._collision_octomap

    @collision_octomap.setter
    def collision_octomap(self, value):
        if __debug__:
            from octomap_msgs.msg import Octomap
            assert \
                isinstance(value, Octomap), \
                "The 'collision_octomap' field must be a sub message of type 'Octomap'"
        self._collision_octomap = value

    @builtins.property
    def elevated_surfel_octomap(self):
        """Message field 'elevated_surfel_octomap'."""
        return self._elevated_surfel_octomap

    @elevated_surfel_octomap.setter
    def elevated_surfel_octomap(self, value):
        if __debug__:
            from octomap_msgs.msg import Octomap
            assert \
                isinstance(value, Octomap), \
                "The 'elevated_surfel_octomap' field must be a sub message of type 'Octomap'"
        self._elevated_surfel_octomap = value

    @builtins.property
    def elevated_surfel_poses(self):
        """Message field 'elevated_surfel_poses'."""
        return self._elevated_surfel_poses

    @elevated_surfel_poses.setter
    def elevated_surfel_poses(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseArray
            assert \
                isinstance(value, PoseArray), \
                "The 'elevated_surfel_poses' field must be a sub message of type 'PoseArray'"
        self._elevated_surfel_poses = value

    @builtins.property
    def traversable_elevated_cloud(self):
        """Message field 'traversable_elevated_cloud'."""
        return self._traversable_elevated_cloud

    @traversable_elevated_cloud.setter
    def traversable_elevated_cloud(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'traversable_elevated_cloud' field must be a sub message of type 'PointCloud2'"
        self._traversable_elevated_cloud = value

    @builtins.property
    def traversable_cloud(self):
        """Message field 'traversable_cloud'."""
        return self._traversable_cloud

    @traversable_cloud.setter
    def traversable_cloud(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'traversable_cloud' field must be a sub message of type 'PointCloud2'"
        self._traversable_cloud = value

    @builtins.property
    def is_valid(self):
        """Message field 'is_valid'."""
        return self._is_valid

    @is_valid.setter
    def is_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_valid' field must be of type 'bool'"
        self._is_valid = value


class Metaclass_GetTraversabilityMap(type):
    """Metaclass of service 'GetTraversabilityMap'."""

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
                'vox_nav_msgs.srv.GetTraversabilityMap')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_traversability_map

            from vox_nav_msgs.srv import _get_traversability_map
            if _get_traversability_map.Metaclass_GetTraversabilityMap_Request._TYPE_SUPPORT is None:
                _get_traversability_map.Metaclass_GetTraversabilityMap_Request.__import_type_support__()
            if _get_traversability_map.Metaclass_GetTraversabilityMap_Response._TYPE_SUPPORT is None:
                _get_traversability_map.Metaclass_GetTraversabilityMap_Response.__import_type_support__()


class GetTraversabilityMap(metaclass=Metaclass_GetTraversabilityMap):
    from vox_nav_msgs.srv._get_traversability_map import GetTraversabilityMap_Request as Request
    from vox_nav_msgs.srv._get_traversability_map import GetTraversabilityMap_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
