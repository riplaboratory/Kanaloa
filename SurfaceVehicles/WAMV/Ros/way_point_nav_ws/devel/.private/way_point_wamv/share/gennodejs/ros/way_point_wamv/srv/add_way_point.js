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

class add_way_pointRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.latitude = null;
      this.longitude = null;
      this.minutes = null;
    }
    else {
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('minutes')) {
        this.minutes = initObj.minutes
      }
      else {
        this.minutes = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type add_way_pointRequest
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [minutes]
    bufferOffset = _serializer.float64(obj.minutes, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type add_way_pointRequest
    let len;
    let data = new add_way_pointRequest(null);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [minutes]
    data.minutes = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'way_point_wamv/add_way_pointRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd43d2876a206169446576ec6e7d81660';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 latitude
    float64 longitude
    float64 minutes
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new add_way_pointRequest(null);
    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.minutes !== undefined) {
      resolved.minutes = msg.minutes;
    }
    else {
      resolved.minutes = 0.0
    }

    return resolved;
    }
};

class add_way_pointResponse {
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
    // Serializes a message object of type add_way_pointResponse
    // Serialize message field [recieved]
    bufferOffset = _serializer.bool(obj.recieved, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type add_way_pointResponse
    let len;
    let data = new add_way_pointResponse(null);
    // Deserialize message field [recieved]
    data.recieved = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'way_point_wamv/add_way_pointResponse';
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
    const resolved = new add_way_pointResponse(null);
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
  Request: add_way_pointRequest,
  Response: add_way_pointResponse,
  md5sum() { return '8f7e4b30056b33ab373baed5e3088955'; },
  datatype() { return 'way_point_wamv/add_way_point'; }
};
