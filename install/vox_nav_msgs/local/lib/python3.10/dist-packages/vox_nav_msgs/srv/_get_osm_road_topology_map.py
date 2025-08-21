# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vox_nav_msgs:srv/GetOSMRoadTopologyMap.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetOSMRoadTopologyMap_Request(type):
    """Metaclass of message 'GetOSMRoadTopologyMap_Request'."""

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
                'vox_nav_msgs.srv.GetOSMRoadTopologyMap_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_osm_road_topology_map__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_osm_road_topology_map__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_osm_road_topology_map__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_osm_road_topology_map__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_osm_road_topology_map__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetOSMRoadTopologyMap_Request(metaclass=Metaclass_GetOSMRoadTopologyMap_Request):
    """Message class 'GetOSMRoadTopologyMap_Request'."""

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


class Metaclass_GetOSMRoadTopologyMap_Response(type):
    """Metaclass of message 'GetOSMRoadTopologyMap_Response'."""

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
                'vox_nav_msgs.srv.GetOSMRoadTopologyMap_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_osm_road_topology_map__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_osm_road_topology_map__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_osm_road_topology_map__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_osm_road_topology_map__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_osm_road_topology_map__response

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


class GetOSMRoadTopologyMap_Response(metaclass=Metaclass_GetOSMRoadTopologyMap_Response):
    """Message class 'GetOSMRoadTopologyMap_Response'."""

    __slots__ = [
        '_osm_road_topology',
        '_is_valid',
    ]

    _fields_and_field_types = {
        'osm_road_topology': 'sensor_msgs/PointCloud2',
        'is_valid': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import PointCloud2
        self.osm_road_topology = kwargs.get('osm_road_topology', PointCloud2())
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
        if self.osm_road_topology != other.osm_road_topology:
            return False
        if self.is_valid != other.is_valid:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def osm_road_topology(self):
        """Message field 'osm_road_topology'."""
        return self._osm_road_topology

    @osm_road_topology.setter
    def osm_road_topology(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'osm_road_topology' field must be a sub message of type 'PointCloud2'"
        self._osm_road_topology = value

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


class Metaclass_GetOSMRoadTopologyMap(type):
    """Metaclass of service 'GetOSMRoadTopologyMap'."""

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
                'vox_nav_msgs.srv.GetOSMRoadTopologyMap')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_osm_road_topology_map

            from vox_nav_msgs.srv import _get_osm_road_topology_map
            if _get_osm_road_topology_map.Metaclass_GetOSMRoadTopologyMap_Request._TYPE_SUPPORT is None:
                _get_osm_road_topology_map.Metaclass_GetOSMRoadTopologyMap_Request.__import_type_support__()
            if _get_osm_road_topology_map.Metaclass_GetOSMRoadTopologyMap_Response._TYPE_SUPPORT is None:
                _get_osm_road_topology_map.Metaclass_GetOSMRoadTopologyMap_Response.__import_type_support__()


class GetOSMRoadTopologyMap(metaclass=Metaclass_GetOSMRoadTopologyMap):
    from vox_nav_msgs.srv._get_osm_road_topology_map import GetOSMRoadTopologyMap_Request as Request
    from vox_nav_msgs.srv._get_osm_road_topology_map import GetOSMRoadTopologyMap_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
