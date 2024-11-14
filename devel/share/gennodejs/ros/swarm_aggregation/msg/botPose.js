// Auto-generated. Do not edit!

// (in-package swarm_aggregation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');

//-----------------------------------------------------------

class botPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bot_id = null;
      this.botpose = null;
    }
    else {
      if (initObj.hasOwnProperty('bot_id')) {
        this.bot_id = initObj.bot_id
      }
      else {
        this.bot_id = [];
      }
      if (initObj.hasOwnProperty('botpose')) {
        this.botpose = initObj.botpose
      }
      else {
        this.botpose = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type botPose
    // Serialize message field [bot_id]
    bufferOffset = _arraySerializer.string(obj.bot_id, buffer, bufferOffset, null);
    // Serialize message field [botpose]
    // Serialize the length for message field [botpose]
    bufferOffset = _serializer.uint32(obj.botpose.length, buffer, bufferOffset);
    obj.botpose.forEach((val) => {
      bufferOffset = nav_msgs.msg.Odometry.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type botPose
    let len;
    let data = new botPose(null);
    // Deserialize message field [bot_id]
    data.bot_id = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [botpose]
    // Deserialize array length for message field [botpose]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.botpose = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.botpose[i] = nav_msgs.msg.Odometry.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.bot_id.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.botpose.forEach((val) => {
      length += nav_msgs.msg.Odometry.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swarm_aggregation/botPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14f9cb1d05d7b333efc6809331fe9a60';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] bot_id
    nav_msgs/Odometry[] botpose
    
    ================================================================================
    MSG: nav_msgs/Odometry
    # This represents an estimate of a position and velocity in free space.  
    # The pose in this message should be specified in the coordinate frame given by header.frame_id.
    # The twist in this message should be specified in the coordinate frame given by the child_frame_id
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
    
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
    MSG: geometry_msgs/PoseWithCovariance
    # This represents a pose in free space with uncertainty.
    
    Pose pose
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/TwistWithCovariance
    # This expresses velocity in free space with uncertainty.
    
    Twist twist
    
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    float64[36] covariance
    
    ================================================================================
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new botPose(null);
    if (msg.bot_id !== undefined) {
      resolved.bot_id = msg.bot_id;
    }
    else {
      resolved.bot_id = []
    }

    if (msg.botpose !== undefined) {
      resolved.botpose = new Array(msg.botpose.length);
      for (let i = 0; i < resolved.botpose.length; ++i) {
        resolved.botpose[i] = nav_msgs.msg.Odometry.Resolve(msg.botpose[i]);
      }
    }
    else {
      resolved.botpose = []
    }

    return resolved;
    }
};

module.exports = botPose;
