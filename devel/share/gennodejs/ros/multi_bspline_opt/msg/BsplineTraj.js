// Auto-generated. Do not edit!

// (in-package multi_bspline_opt.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
<<<<<<< HEAD
let std_msgs = _finder('std_msgs');
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd

//-----------------------------------------------------------

class BsplineTraj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
<<<<<<< HEAD
      this.header = null;
      this.position = null;
      this.velocity = null;
      this.acceleration = null;
      this.yaw = null;
      this.yaw_rate = null;
      this.current_seq = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = [];
=======
      this.drone_id = null;
      this.traj_id = null;
      this.start_time = null;
      this.duration = null;
      this.yaw = null;
      this.yaw_rate = null;
      this.position = null;
      this.velocity = null;
      this.acceleration = null;
      this.current_seq = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('traj_id')) {
        this.traj_id = initObj.traj_id
      }
      else {
        this.traj_id = 0;
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate')) {
        this.yaw_rate = initObj.yaw_rate
      }
      else {
        this.yaw_rate = 0.0;
      }
<<<<<<< HEAD
=======
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = [];
      }
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
      if (initObj.hasOwnProperty('current_seq')) {
        this.current_seq = initObj.current_seq
      }
      else {
        this.current_seq = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BsplineTraj
<<<<<<< HEAD
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
=======
    // Serialize message field [drone_id]
    bufferOffset = _serializer.int32(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [traj_id]
    bufferOffset = _serializer.int64(obj.traj_id, buffer, bufferOffset);
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float64(obj.duration, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float64(obj.yaw_rate, buffer, bufferOffset);
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    // Serialize message field [position]
    // Serialize the length for message field [position]
    bufferOffset = _serializer.uint32(obj.position.length, buffer, bufferOffset);
    obj.position.forEach((val) => {
<<<<<<< HEAD
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
=======
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    });
    // Serialize message field [velocity]
    // Serialize the length for message field [velocity]
    bufferOffset = _serializer.uint32(obj.velocity.length, buffer, bufferOffset);
    obj.velocity.forEach((val) => {
<<<<<<< HEAD
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
=======
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    });
    // Serialize message field [acceleration]
    // Serialize the length for message field [acceleration]
    bufferOffset = _serializer.uint32(obj.acceleration.length, buffer, bufferOffset);
    obj.acceleration.forEach((val) => {
<<<<<<< HEAD
      bufferOffset = geometry_msgs.msg.PoseStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float32(obj.yaw_rate, buffer, bufferOffset);
=======
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    // Serialize message field [current_seq]
    bufferOffset = _serializer.uint32(obj.current_seq, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BsplineTraj
    let len;
    let data = new BsplineTraj(null);
<<<<<<< HEAD
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
=======
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [traj_id]
    data.traj_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float64(buffer, bufferOffset);
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    // Deserialize message field [position]
    // Deserialize array length for message field [position]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.position = new Array(len);
    for (let i = 0; i < len; ++i) {
<<<<<<< HEAD
      data.position[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
=======
      data.position[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    }
    // Deserialize message field [velocity]
    // Deserialize array length for message field [velocity]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.velocity = new Array(len);
    for (let i = 0; i < len; ++i) {
<<<<<<< HEAD
      data.velocity[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
=======
      data.velocity[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    }
    // Deserialize message field [acceleration]
    // Deserialize array length for message field [acceleration]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.acceleration = new Array(len);
    for (let i = 0; i < len; ++i) {
<<<<<<< HEAD
      data.acceleration[i] = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float32(buffer, bufferOffset);
=======
      data.acceleration[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    // Deserialize message field [current_seq]
    data.current_seq = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
<<<<<<< HEAD
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.position.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    object.velocity.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    object.acceleration.forEach((val) => {
      length += geometry_msgs.msg.PoseStamped.getMessageSize(val);
    });
    return length + 24;
=======
    length += 24 * object.position.length;
    length += 24 * object.velocity.length;
    length += 24 * object.acceleration.length;
    return length + 60;
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
  }

  static datatype() {
    // Returns string type for a message object
    return 'multi_bspline_opt/BsplineTraj';
  }

  static md5sum() {
    //Returns md5sum for a message object
<<<<<<< HEAD
    return '47fe784cc6ad92a6cd7d1488e4d545d9';
=======
    return 'c277c0fe692a3c44cb39de5d41a0fe4c';
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
<<<<<<< HEAD
    std_msgs/Header header
    
    geometry_msgs/PoseStamped[] position
    geometry_msgs/PoseStamped[] velocity
    geometry_msgs/PoseStamped[] acceleration
    
    float32 yaw
    float32 yaw_rate
    uint32 current_seq
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
=======
    # std_msgs/Header header
    
    int32 drone_id
    int64 traj_id
    time start_time
    float64 duration
    
    float64 yaw
    float64 yaw_rate
    
    geometry_msgs/Point[] position
    geometry_msgs/Point[] velocity
    geometry_msgs/Point[] acceleration
    
    uint32 current_seq
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
<<<<<<< HEAD
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BsplineTraj(null);
<<<<<<< HEAD
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
=======
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.traj_id !== undefined) {
      resolved.traj_id = msg.traj_id;
    }
    else {
      resolved.traj_id = 0
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.yaw_rate !== undefined) {
      resolved.yaw_rate = msg.yaw_rate;
    }
    else {
      resolved.yaw_rate = 0.0
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    }

    if (msg.position !== undefined) {
      resolved.position = new Array(msg.position.length);
      for (let i = 0; i < resolved.position.length; ++i) {
<<<<<<< HEAD
        resolved.position[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.position[i]);
=======
        resolved.position[i] = geometry_msgs.msg.Point.Resolve(msg.position[i]);
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
      }
    }
    else {
      resolved.position = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = new Array(msg.velocity.length);
      for (let i = 0; i < resolved.velocity.length; ++i) {
<<<<<<< HEAD
        resolved.velocity[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.velocity[i]);
=======
        resolved.velocity[i] = geometry_msgs.msg.Point.Resolve(msg.velocity[i]);
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
      }
    }
    else {
      resolved.velocity = []
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = new Array(msg.acceleration.length);
      for (let i = 0; i < resolved.acceleration.length; ++i) {
<<<<<<< HEAD
        resolved.acceleration[i] = geometry_msgs.msg.PoseStamped.Resolve(msg.acceleration[i]);
=======
        resolved.acceleration[i] = geometry_msgs.msg.Point.Resolve(msg.acceleration[i]);
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
      }
    }
    else {
      resolved.acceleration = []
    }

<<<<<<< HEAD
    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.yaw_rate !== undefined) {
      resolved.yaw_rate = msg.yaw_rate;
    }
    else {
      resolved.yaw_rate = 0.0
    }

=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    if (msg.current_seq !== undefined) {
      resolved.current_seq = msg.current_seq;
    }
    else {
      resolved.current_seq = 0
    }

    return resolved;
    }
};

module.exports = BsplineTraj;
