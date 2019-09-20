// Auto-generated. Do not edit!

// (in-package way_point_wamv.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class way_point_cmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type way_point_cmdRequest
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type way_point_cmdRequest
    let len;
    let data = new way_point_cmdRequest(null);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.command.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'way_point_wamv/way_point_cmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cba5e21e920a3a2b7b375cb65b64cdea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string command
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new way_point_cmdRequest(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    return resolved;
    }
};

class way_point_cmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.recieved = null;
    }
    else {
      if (initObj.hasOwnProperty('recieved')) {
        this.recieved = initObj.recieved
      }
      else {
        this.recieved = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type way_point_cmdResponse
    // Serialize message field [recieved]
    bufferOffset = _serializer.bool(obj.recieved, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type way_point_cmdResponse
    let len;
    let data = new way_point_cmdResponse(null);
    // Deserialize message field [recieved]
    data.recieved = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'way_point_wamv/way_point_cmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7dab0b7be2e21c7e76041e8de101b25f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool recieved
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new way_point_cmdResponse(null);
    if (msg.recieved !== undefined) {
      resolved.recieved = msg.recieved;
    }
    else {
      resolved.recieved = false
    }

    return resolved;
    }
};

module.exports = {
  Request: way_point_cmdRequest,
  Response: way_point_cmdResponse,
  md5sum() { return '9115d0e027e87d5bbec3f644195b57c4'; },
  datatype() { return 'way_point_wamv/way_point_cmd'; }
};
