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

//-----------------------------------------------------------

class SendTraj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.traj_id = null;
      this.order = null;
      this.cps_num_ = null;
      this.start_time = null;
      this.start_pos_ = null;
      this.start_vel_ = null;
      this.start_acc_ = null;
      this.end_pos_ = null;
      this.control_pts = null;
      this.knots = null;
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
      if (initObj.hasOwnProperty('order')) {
        this.order = initObj.order
      }
      else {
        this.order = 0;
      }
      if (initObj.hasOwnProperty('cps_num_')) {
        this.cps_num_ = initObj.cps_num_
      }
      else {
        this.cps_num_ = 0;
      }
      if (initObj.hasOwnProperty('start_time')) {
        this.start_time = initObj.start_time
      }
      else {
        this.start_time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('start_pos_')) {
        this.start_pos_ = initObj.start_pos_
      }
      else {
        this.start_pos_ = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('start_vel_')) {
        this.start_vel_ = initObj.start_vel_
      }
      else {
        this.start_vel_ = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('start_acc_')) {
        this.start_acc_ = initObj.start_acc_
      }
      else {
        this.start_acc_ = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('end_pos_')) {
        this.end_pos_ = initObj.end_pos_
      }
      else {
        this.end_pos_ = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('control_pts')) {
        this.control_pts = initObj.control_pts
      }
      else {
        this.control_pts = [];
      }
      if (initObj.hasOwnProperty('knots')) {
        this.knots = initObj.knots
      }
      else {
        this.knots = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SendTraj
    // Serialize message field [drone_id]
    bufferOffset = _serializer.int32(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [traj_id]
    bufferOffset = _serializer.int64(obj.traj_id, buffer, bufferOffset);
    // Serialize message field [order]
    bufferOffset = _serializer.int32(obj.order, buffer, bufferOffset);
    // Serialize message field [cps_num_]
    bufferOffset = _serializer.int32(obj.cps_num_, buffer, bufferOffset);
    // Serialize message field [start_time]
    bufferOffset = _serializer.time(obj.start_time, buffer, bufferOffset);
    // Serialize message field [start_pos_]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.start_pos_, buffer, bufferOffset);
    // Serialize message field [start_vel_]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.start_vel_, buffer, bufferOffset);
    // Serialize message field [start_acc_]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.start_acc_, buffer, bufferOffset);
    // Serialize message field [end_pos_]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.end_pos_, buffer, bufferOffset);
    // Serialize message field [control_pts]
    // Serialize the length for message field [control_pts]
    bufferOffset = _serializer.uint32(obj.control_pts.length, buffer, bufferOffset);
    obj.control_pts.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [knots]
    bufferOffset = _arraySerializer.float64(obj.knots, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SendTraj
    let len;
    let data = new SendTraj(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [traj_id]
    data.traj_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [order]
    data.order = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [cps_num_]
    data.cps_num_ = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [start_time]
    data.start_time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [start_pos_]
    data.start_pos_ = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [start_vel_]
    data.start_vel_ = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [start_acc_]
    data.start_acc_ = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_pos_]
    data.end_pos_ = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [control_pts]
    // Deserialize array length for message field [control_pts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.control_pts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.control_pts[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [knots]
    data.knots = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.control_pts.length;
    length += 8 * object.knots.length;
    return length + 132;
  }

  static datatype() {
    // Returns string type for a message object
    return 'multi_bspline_opt/SendTraj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd1c07ca91141848b23c4fb9cb569a4f8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 drone_id
    int64 traj_id
    int32 order
    int32 cps_num_
    # int32 Dim_
    # int32 TrajSampleRate
    # float64 beta
    time start_time
    
    
    geometry_msgs/Point start_pos_
    geometry_msgs/Point start_vel_
    geometry_msgs/Point start_acc_
    geometry_msgs/Point end_pos_
    # float64 start_pos_x
    # float64 start_pos_y
    # float64 start_vel_x
    # float64 start_vel_y
    # float64 start_acc_x
    # float64 start_acc_y
    # float64 end_pos_x
    # float64 end_pos_y
    # float64 yaw_rate
    
    geometry_msgs/Point[] control_pts
    float64[] knots
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SendTraj(null);
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

    if (msg.order !== undefined) {
      resolved.order = msg.order;
    }
    else {
      resolved.order = 0
    }

    if (msg.cps_num_ !== undefined) {
      resolved.cps_num_ = msg.cps_num_;
    }
    else {
      resolved.cps_num_ = 0
    }

    if (msg.start_time !== undefined) {
      resolved.start_time = msg.start_time;
    }
    else {
      resolved.start_time = {secs: 0, nsecs: 0}
    }

    if (msg.start_pos_ !== undefined) {
      resolved.start_pos_ = geometry_msgs.msg.Point.Resolve(msg.start_pos_)
    }
    else {
      resolved.start_pos_ = new geometry_msgs.msg.Point()
    }

    if (msg.start_vel_ !== undefined) {
      resolved.start_vel_ = geometry_msgs.msg.Point.Resolve(msg.start_vel_)
    }
    else {
      resolved.start_vel_ = new geometry_msgs.msg.Point()
    }

    if (msg.start_acc_ !== undefined) {
      resolved.start_acc_ = geometry_msgs.msg.Point.Resolve(msg.start_acc_)
    }
    else {
      resolved.start_acc_ = new geometry_msgs.msg.Point()
    }

    if (msg.end_pos_ !== undefined) {
      resolved.end_pos_ = geometry_msgs.msg.Point.Resolve(msg.end_pos_)
    }
    else {
      resolved.end_pos_ = new geometry_msgs.msg.Point()
    }

    if (msg.control_pts !== undefined) {
      resolved.control_pts = new Array(msg.control_pts.length);
      for (let i = 0; i < resolved.control_pts.length; ++i) {
        resolved.control_pts[i] = geometry_msgs.msg.Point.Resolve(msg.control_pts[i]);
      }
    }
    else {
      resolved.control_pts = []
    }

    if (msg.knots !== undefined) {
      resolved.knots = msg.knots;
    }
    else {
      resolved.knots = []
    }

    return resolved;
    }
};

module.exports = SendTraj;
