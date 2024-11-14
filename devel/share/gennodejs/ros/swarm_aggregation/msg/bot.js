// Auto-generated. Do not edit!

// (in-package swarm_aggregation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class bot {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.pose_x = null;
      this.pose_y = null;
      this.pose_theta = null;
      this.vel_x = null;
      this.vel_y = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('pose_x')) {
        this.pose_x = initObj.pose_x
      }
      else {
        this.pose_x = 0.0;
      }
      if (initObj.hasOwnProperty('pose_y')) {
        this.pose_y = initObj.pose_y
      }
      else {
        this.pose_y = 0.0;
      }
      if (initObj.hasOwnProperty('pose_theta')) {
        this.pose_theta = initObj.pose_theta
      }
      else {
        this.pose_theta = 0.0;
      }
      if (initObj.hasOwnProperty('vel_x')) {
        this.vel_x = initObj.vel_x
      }
      else {
        this.vel_x = 0.0;
      }
      if (initObj.hasOwnProperty('vel_y')) {
        this.vel_y = initObj.vel_y
      }
      else {
        this.vel_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type bot
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [pose_x]
    bufferOffset = _serializer.float64(obj.pose_x, buffer, bufferOffset);
    // Serialize message field [pose_y]
    bufferOffset = _serializer.float64(obj.pose_y, buffer, bufferOffset);
    // Serialize message field [pose_theta]
    bufferOffset = _serializer.float64(obj.pose_theta, buffer, bufferOffset);
    // Serialize message field [vel_x]
    bufferOffset = _serializer.float64(obj.vel_x, buffer, bufferOffset);
    // Serialize message field [vel_y]
    bufferOffset = _serializer.float64(obj.vel_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type bot
    let len;
    let data = new bot(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pose_x]
    data.pose_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pose_y]
    data.pose_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pose_theta]
    data.pose_theta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_x]
    data.vel_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_y]
    data.vel_y = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.id);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'swarm_aggregation/bot';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4cb3c0cf91cbe6f044efb99c3a3f8cdf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string id
    float64 pose_x
    float64 pose_y
    float64 pose_theta
    float64 vel_x
    float64 vel_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new bot(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.pose_x !== undefined) {
      resolved.pose_x = msg.pose_x;
    }
    else {
      resolved.pose_x = 0.0
    }

    if (msg.pose_y !== undefined) {
      resolved.pose_y = msg.pose_y;
    }
    else {
      resolved.pose_y = 0.0
    }

    if (msg.pose_theta !== undefined) {
      resolved.pose_theta = msg.pose_theta;
    }
    else {
      resolved.pose_theta = 0.0
    }

    if (msg.vel_x !== undefined) {
      resolved.vel_x = msg.vel_x;
    }
    else {
      resolved.vel_x = 0.0
    }

    if (msg.vel_y !== undefined) {
      resolved.vel_y = msg.vel_y;
    }
    else {
      resolved.vel_y = 0.0
    }

    return resolved;
    }
};

module.exports = bot;
