// Auto-generated. Do not edit!

// (in-package robotx_gazebo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class UsvDrive {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left = null;
      this.right = null;
    }
    else {
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = 0.0;
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UsvDrive
    // Serialize message field [left]
    bufferOffset = _serializer.float32(obj.left, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = _serializer.float32(obj.right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UsvDrive
    let len;
    let data = new UsvDrive(null);
    // Deserialize message field [left]
    data.left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotx_gazebo/UsvDrive';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3a927990ab5d5c3d628e2d52b8533e52';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Standard USV thrust massage - port and starbard
    
    # Thrust command - typically ranges from {-1.0 - 1.0}
    float32 left
    float32 right
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UsvDrive(null);
    if (msg.left !== undefined) {
      resolved.left = msg.left;
    }
    else {
      resolved.left = 0.0
    }

    if (msg.right !== undefined) {
      resolved.right = msg.right;
    }
    else {
      resolved.right = 0.0
    }

    return resolved;
    }
};

module.exports = UsvDrive;
