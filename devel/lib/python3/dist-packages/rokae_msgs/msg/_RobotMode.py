# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from rokae_msgs/RobotMode.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class RobotMode(genpy.Message):
  _md5sum = "37b92b84717583128a825ed4248ac20e"
  _type = "rokae_msgs/RobotMode"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# This is for robot state mode structure
float64 timestamp
bool is_robot_connected
bool is_real_robot_enabled
bool is_robot_power_on
bool is_robot_running
bool is_program_running
bool is_emergency_stopped"""
  __slots__ = ['timestamp','is_robot_connected','is_real_robot_enabled','is_robot_power_on','is_robot_running','is_program_running','is_emergency_stopped']
  _slot_types = ['float64','bool','bool','bool','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       timestamp,is_robot_connected,is_real_robot_enabled,is_robot_power_on,is_robot_running,is_program_running,is_emergency_stopped

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(RobotMode, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.timestamp is None:
        self.timestamp = 0.
      if self.is_robot_connected is None:
        self.is_robot_connected = False
      if self.is_real_robot_enabled is None:
        self.is_real_robot_enabled = False
      if self.is_robot_power_on is None:
        self.is_robot_power_on = False
      if self.is_robot_running is None:
        self.is_robot_running = False
      if self.is_program_running is None:
        self.is_program_running = False
      if self.is_emergency_stopped is None:
        self.is_emergency_stopped = False
    else:
      self.timestamp = 0.
      self.is_robot_connected = False
      self.is_real_robot_enabled = False
      self.is_robot_power_on = False
      self.is_robot_running = False
      self.is_program_running = False
      self.is_emergency_stopped = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_d6B().pack(_x.timestamp, _x.is_robot_connected, _x.is_real_robot_enabled, _x.is_robot_power_on, _x.is_robot_running, _x.is_program_running, _x.is_emergency_stopped))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 14
      (_x.timestamp, _x.is_robot_connected, _x.is_real_robot_enabled, _x.is_robot_power_on, _x.is_robot_running, _x.is_program_running, _x.is_emergency_stopped,) = _get_struct_d6B().unpack(str[start:end])
      self.is_robot_connected = bool(self.is_robot_connected)
      self.is_real_robot_enabled = bool(self.is_real_robot_enabled)
      self.is_robot_power_on = bool(self.is_robot_power_on)
      self.is_robot_running = bool(self.is_robot_running)
      self.is_program_running = bool(self.is_program_running)
      self.is_emergency_stopped = bool(self.is_emergency_stopped)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_d6B().pack(_x.timestamp, _x.is_robot_connected, _x.is_real_robot_enabled, _x.is_robot_power_on, _x.is_robot_running, _x.is_program_running, _x.is_emergency_stopped))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 14
      (_x.timestamp, _x.is_robot_connected, _x.is_real_robot_enabled, _x.is_robot_power_on, _x.is_robot_running, _x.is_program_running, _x.is_emergency_stopped,) = _get_struct_d6B().unpack(str[start:end])
      self.is_robot_connected = bool(self.is_robot_connected)
      self.is_real_robot_enabled = bool(self.is_real_robot_enabled)
      self.is_robot_power_on = bool(self.is_robot_power_on)
      self.is_robot_running = bool(self.is_robot_running)
      self.is_program_running = bool(self.is_program_running)
      self.is_emergency_stopped = bool(self.is_emergency_stopped)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_d6B = None
def _get_struct_d6B():
    global _struct_d6B
    if _struct_d6B is None:
        _struct_d6B = struct.Struct("<d6B")
    return _struct_d6B
