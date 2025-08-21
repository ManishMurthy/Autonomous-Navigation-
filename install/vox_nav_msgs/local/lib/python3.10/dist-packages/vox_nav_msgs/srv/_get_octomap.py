# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vox_nav_msgs:srv/GetOctomap.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetOctomap_Request(type):
    """Metaclass of message 'GetOctomap_Request'."""

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
                'vox_nav_msgs.srv.GetOctomap_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_octomap__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_octomap__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_octomap__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_octomap__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_octomap__request

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetOctomap_Request(metaclass=Metaclass_GetOctomap_Request):
    """Message class 'GetOctomap_Request'."""

    __slots__ = [
        '_bounding_box_origin',
        '_bounding_box_lengths',
        '_leaf_size',
        '_publish_octomap',
        '_filename',
    ]

    _fields_and_field_types = {
        'bounding_box_origin': 'geometry_msgs/Point',
        'bounding_box_lengths': 'geometry_msgs/Point',
        'leaf_size': 'double',
        'publish_octomap': 'boolean',
        'filename': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Point
        self.bounding_box_origin = kwargs.get('bounding_box_origin', Point())
        from geometry_msgs.msg import Point
        self.bounding_box_lengths = kwargs.get('bounding_box_lengths', Point())
        self.leaf_size = kwargs.get('leaf_size', float())
        self.publish_octomap = kwargs.get('publish_octomap', bool())
        self.filename = kwargs.get('filename', str())

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
        if self.bounding_box_origin != other.bounding_box_origin:
            return False
        if self.bounding_box_lengths != other.bounding_box_lengths:
            return False
        if self.leaf_size != other.leaf_size:
            return False
        if self.publish_octomap != other.publish_octomap:
            return False
        if self.filename != other.filename:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def bounding_box_origin(self):
        """Message field 'bounding_box_origin'."""
        return self._bounding_box_origin

    @bounding_box_origin.setter
    def bounding_box_origin(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'bounding_box_origin' field must be a sub message of type 'Point'"
        self._bounding_box_origin = value

    @builtins.property
    def bounding_box_lengths(self):
        """Message field 'bounding_box_lengths'."""
        return self._bounding_box_lengths

    @bounding_box_lengths.setter
    def bounding_box_lengths(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'bounding_box_lengths' field must be a sub message of type 'Point'"
        self._bounding_box_lengths = value

    @builtins.property
    def leaf_size(self):
        """Message field 'leaf_size'."""
        return self._leaf_size

    @leaf_size.setter
    def leaf_size(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'leaf_size' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'leaf_size' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._leaf_size = value

    @builtins.property
    def publish_octomap(self):
        """Message field 'publish_octomap'."""
        return self._publish_octomap

    @publish_octomap.setter
    def publish_octomap(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'publish_octomap' field must be of type 'bool'"
        self._publish_octomap = value

    @builtins.property
    def filename(self):
        """Message field 'filename'."""
        return self._filename

    @filename.setter
    def filename(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'filename' field must be of type 'str'"
        self._filename = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_GetOctomap_Response(type):
    """Metaclass of message 'GetOctomap_Response'."""

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
                'vox_nav_msgs.srv.GetOctomap_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_octomap__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_octomap__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_octomap__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_octomap__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_octomap__response

            from octomap_msgs.msg import Octomap
            if Octomap.__class__._TYPE_SUPPORT is None:
                Octomap.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetOctomap_Response(metaclass=Metaclass_GetOctomap_Response):
    """Message class 'GetOctomap_Response'."""

    __slots__ = [
        '_map',
        '_origin_latitude',
        '_origin_longitude',
        '_origin_altitude',
    ]

    _fields_and_field_types = {
        'map': 'octomap_msgs/Octomap',
        'origin_latitude': 'double',
        'origin_longitude': 'double',
        'origin_altitude': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['octomap_msgs', 'msg'], 'Octomap'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from octomap_msgs.msg import Octomap
        self.map = kwargs.get('map', Octomap())
        self.origin_latitude = kwargs.get('origin_latitude', float())
        self.origin_longitude = kwargs.get('origin_longitude', float())
        self.origin_altitude = kwargs.get('origin_altitude', float())

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
        if self.map != other.map:
            return False
        if self.origin_latitude != other.origin_latitude:
            return False
        if self.origin_longitude != other.origin_longitude:
            return False
        if self.origin_altitude != other.origin_altitude:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def map(self):  # noqa: A003
        """Message field 'map'."""
        return self._map

    @map.setter  # noqa: A003
    def map(self, value):  # noqa: A003
        if __debug__:
            from octomap_msgs.msg import Octomap
            assert \
                isinstance(value, Octomap), \
                "The 'map' field must be a sub message of type 'Octomap'"
        self._map = value

    @builtins.property
    def origin_latitude(self):
        """Message field 'origin_latitude'."""
        return self._origin_latitude

    @origin_latitude.setter
    def origin_latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'origin_latitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'origin_latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._origin_latitude = value

    @builtins.property
    def origin_longitude(self):
        """Message field 'origin_longitude'."""
        return self._origin_longitude

    @origin_longitude.setter
    def origin_longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'origin_longitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'origin_longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._origin_longitude = value

    @builtins.property
    def origin_altitude(self):
        """Message field 'origin_altitude'."""
        return self._origin_altitude

    @origin_altitude.setter
    def origin_altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'origin_altitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'origin_altitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._origin_altitude = value


class Metaclass_GetOctomap(type):
    """Metaclass of service 'GetOctomap'."""

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
                'vox_nav_msgs.srv.GetOctomap')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_octomap

            from vox_nav_msgs.srv import _get_octomap
            if _get_octomap.Metaclass_GetOctomap_Request._TYPE_SUPPORT is None:
                _get_octomap.Metaclass_GetOctomap_Request.__import_type_support__()
            if _get_octomap.Metaclass_GetOctomap_Response._TYPE_SUPPORT is None:
                _get_octomap.Metaclass_GetOctomap_Response.__import_type_support__()


class GetOctomap(metaclass=Metaclass_GetOctomap):
    from vox_nav_msgs.srv._get_octomap import GetOctomap_Request as Request
    from vox_nav_msgs.srv._get_octomap import GetOctomap_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
