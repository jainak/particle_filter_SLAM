"""autogenerated by genpy from pf_drone_state_estimation/Feature_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import pf_drone_state_estimation.msg
import std_msgs.msg

class Feature_msg(genpy.Message):
  _md5sum = "140431f548ab202fcb69c43ce5985859"
  _type = "pf_drone_state_estimation/Feature_msg"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

float64 px
float64 py

Feature_Keypoint f

float64 posX
float64 posY
float64 posZ

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: pf_drone_state_estimation/Feature_Keypoint
Header header

float64 x
float64 y

float64 size
float64 angle
float64 response
int32 octave
int32 class_id


"""
  __slots__ = ['header','px','py','f','posX','posY','posZ']
  _slot_types = ['std_msgs/Header','float64','float64','pf_drone_state_estimation/Feature_Keypoint','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,px,py,f,posX,posY,posZ

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Feature_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.px is None:
        self.px = 0.
      if self.py is None:
        self.py = 0.
      if self.f is None:
        self.f = pf_drone_state_estimation.msg.Feature_Keypoint()
      if self.posX is None:
        self.posX = 0.
      if self.posY is None:
        self.posY = 0.
      if self.posZ is None:
        self.posZ = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.px = 0.
      self.py = 0.
      self.f = pf_drone_state_estimation.msg.Feature_Keypoint()
      self.posX = 0.
      self.posY = 0.
      self.posZ = 0.

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d3I.pack(_x.px, _x.py, _x.f.header.seq, _x.f.header.stamp.secs, _x.f.header.stamp.nsecs))
      _x = self.f.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_5d2i3d.pack(_x.f.x, _x.f.y, _x.f.size, _x.f.angle, _x.f.response, _x.f.octave, _x.f.class_id, _x.posX, _x.posY, _x.posZ))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.f is None:
        self.f = pf_drone_state_estimation.msg.Feature_Keypoint()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 28
      (_x.px, _x.py, _x.f.header.seq, _x.f.header.stamp.secs, _x.f.header.stamp.nsecs,) = _struct_2d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.f.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.f.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 72
      (_x.f.x, _x.f.y, _x.f.size, _x.f.angle, _x.f.response, _x.f.octave, _x.f.class_id, _x.posX, _x.posY, _x.posZ,) = _struct_5d2i3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_2d3I.pack(_x.px, _x.py, _x.f.header.seq, _x.f.header.stamp.secs, _x.f.header.stamp.nsecs))
      _x = self.f.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_5d2i3d.pack(_x.f.x, _x.f.y, _x.f.size, _x.f.angle, _x.f.response, _x.f.octave, _x.f.class_id, _x.posX, _x.posY, _x.posZ))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.f is None:
        self.f = pf_drone_state_estimation.msg.Feature_Keypoint()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 28
      (_x.px, _x.py, _x.f.header.seq, _x.f.header.stamp.secs, _x.f.header.stamp.nsecs,) = _struct_2d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.f.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.f.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 72
      (_x.f.x, _x.f.y, _x.f.size, _x.f.angle, _x.f.response, _x.f.octave, _x.f.class_id, _x.posX, _x.posY, _x.posZ,) = _struct_5d2i3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_5d2i3d = struct.Struct("<5d2i3d")
_struct_2d3I = struct.Struct("<2d3I")