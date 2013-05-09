"""autogenerated by genpy from pf_drone_state_estimation/Measurement_data.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import pf_drone_state_estimation.msg
import std_msgs.msg

class Measurement_data(genpy.Message):
  _md5sum = "3811ed9c9a95cff36a423c1adf3afc4c"
  _type = "pf_drone_state_estimation/Measurement_data"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

Feature_msg[] features

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
MSG: pf_drone_state_estimation/Feature_msg
Header header

float64 px
float64 py

Feature_Keypoint f

float64 posX
float64 posY
float64 posZ

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
  __slots__ = ['header','features']
  _slot_types = ['std_msgs/Header','pf_drone_state_estimation/Feature_msg[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,features

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Measurement_data, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.features is None:
        self.features = []
    else:
      self.header = std_msgs.msg.Header()
      self.features = []

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
      length = len(self.features)
      buff.write(_struct_I.pack(length))
      for val1 in self.features:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2d.pack(_x.px, _x.py))
        _v3 = val1.f
        _v4 = _v3.header
        buff.write(_struct_I.pack(_v4.seq))
        _v5 = _v4.stamp
        _x = _v5
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v4.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = _v3
        buff.write(_struct_5d2i.pack(_x.x, _x.y, _x.size, _x.angle, _x.response, _x.octave, _x.class_id))
        _x = val1
        buff.write(_struct_3d.pack(_x.posX, _x.posY, _x.posZ))
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
      if self.features is None:
        self.features = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.features = []
      for i in range(0, length):
        val1 = pf_drone_state_estimation.msg.Feature_msg()
        _v6 = val1.header
        start = end
        end += 4
        (_v6.seq,) = _struct_I.unpack(str[start:end])
        _v7 = _v6.stamp
        _x = _v7
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v6.frame_id = str[start:end].decode('utf-8')
        else:
          _v6.frame_id = str[start:end]
        _x = val1
        start = end
        end += 16
        (_x.px, _x.py,) = _struct_2d.unpack(str[start:end])
        _v8 = val1.f
        _v9 = _v8.header
        start = end
        end += 4
        (_v9.seq,) = _struct_I.unpack(str[start:end])
        _v10 = _v9.stamp
        _x = _v10
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v9.frame_id = str[start:end].decode('utf-8')
        else:
          _v9.frame_id = str[start:end]
        _x = _v8
        start = end
        end += 48
        (_x.x, _x.y, _x.size, _x.angle, _x.response, _x.octave, _x.class_id,) = _struct_5d2i.unpack(str[start:end])
        _x = val1
        start = end
        end += 24
        (_x.posX, _x.posY, _x.posZ,) = _struct_3d.unpack(str[start:end])
        self.features.append(val1)
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
      length = len(self.features)
      buff.write(_struct_I.pack(length))
      for val1 in self.features:
        _v11 = val1.header
        buff.write(_struct_I.pack(_v11.seq))
        _v12 = _v11.stamp
        _x = _v12
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v11.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_2d.pack(_x.px, _x.py))
        _v13 = val1.f
        _v14 = _v13.header
        buff.write(_struct_I.pack(_v14.seq))
        _v15 = _v14.stamp
        _x = _v15
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v14.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = _v13
        buff.write(_struct_5d2i.pack(_x.x, _x.y, _x.size, _x.angle, _x.response, _x.octave, _x.class_id))
        _x = val1
        buff.write(_struct_3d.pack(_x.posX, _x.posY, _x.posZ))
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
      if self.features is None:
        self.features = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.features = []
      for i in range(0, length):
        val1 = pf_drone_state_estimation.msg.Feature_msg()
        _v16 = val1.header
        start = end
        end += 4
        (_v16.seq,) = _struct_I.unpack(str[start:end])
        _v17 = _v16.stamp
        _x = _v17
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v16.frame_id = str[start:end].decode('utf-8')
        else:
          _v16.frame_id = str[start:end]
        _x = val1
        start = end
        end += 16
        (_x.px, _x.py,) = _struct_2d.unpack(str[start:end])
        _v18 = val1.f
        _v19 = _v18.header
        start = end
        end += 4
        (_v19.seq,) = _struct_I.unpack(str[start:end])
        _v20 = _v19.stamp
        _x = _v20
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v19.frame_id = str[start:end].decode('utf-8')
        else:
          _v19.frame_id = str[start:end]
        _x = _v18
        start = end
        end += 48
        (_x.x, _x.y, _x.size, _x.angle, _x.response, _x.octave, _x.class_id,) = _struct_5d2i.unpack(str[start:end])
        _x = val1
        start = end
        end += 24
        (_x.posX, _x.posY, _x.posZ,) = _struct_3d.unpack(str[start:end])
        self.features.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_5d2i = struct.Struct("<5d2i")
_struct_2d = struct.Struct("<2d")
_struct_3I = struct.Struct("<3I")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")
