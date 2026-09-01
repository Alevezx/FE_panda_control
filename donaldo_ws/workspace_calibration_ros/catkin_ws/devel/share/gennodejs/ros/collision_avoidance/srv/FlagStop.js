// Auto-generated. Do not edit!

// (in-package collision_avoidance.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class FlagStopRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.T_stop = null;
      this.q = null;
      this.q_p = null;
      this.q_pp = null;
    }
    else {
      if (initObj.hasOwnProperty('T_stop')) {
        this.T_stop = initObj.T_stop
      }
      else {
        this.T_stop = 0.0;
      }
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = [];
      }
      if (initObj.hasOwnProperty('q_p')) {
        this.q_p = initObj.q_p
      }
      else {
        this.q_p = [];
      }
      if (initObj.hasOwnProperty('q_pp')) {
        this.q_pp = initObj.q_pp
      }
      else {
        this.q_pp = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlagStopRequest
    // Serialize message field [T_stop]
    bufferOffset = _serializer.float64(obj.T_stop, buffer, bufferOffset);
    // Serialize message field [q]
    bufferOffset = _arraySerializer.float64(obj.q, buffer, bufferOffset, null);
    // Serialize message field [q_p]
    bufferOffset = _arraySerializer.float64(obj.q_p, buffer, bufferOffset, null);
    // Serialize message field [q_pp]
    bufferOffset = _arraySerializer.float64(obj.q_pp, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlagStopRequest
    let len;
    let data = new FlagStopRequest(null);
    // Deserialize message field [T_stop]
    data.T_stop = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [q]
    data.q = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [q_p]
    data.q_p = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [q_pp]
    data.q_pp = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.q.length;
    length += 8 * object.q_p.length;
    length += 8 * object.q_pp.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'collision_avoidance/FlagStopRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e024533f10287e7f7806c4581eb65396';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 T_stop
    float64[] q
    float64[] q_p
    float64[] q_pp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlagStopRequest(null);
    if (msg.T_stop !== undefined) {
      resolved.T_stop = msg.T_stop;
    }
    else {
      resolved.T_stop = 0.0
    }

    if (msg.q !== undefined) {
      resolved.q = msg.q;
    }
    else {
      resolved.q = []
    }

    if (msg.q_p !== undefined) {
      resolved.q_p = msg.q_p;
    }
    else {
      resolved.q_p = []
    }

    if (msg.q_pp !== undefined) {
      resolved.q_pp = msg.q_pp;
    }
    else {
      resolved.q_pp = []
    }

    return resolved;
    }
};

class FlagStopResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flag_stop = null;
    }
    else {
      if (initObj.hasOwnProperty('flag_stop')) {
        this.flag_stop = initObj.flag_stop
      }
      else {
        this.flag_stop = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlagStopResponse
    // Serialize message field [flag_stop]
    bufferOffset = _serializer.bool(obj.flag_stop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlagStopResponse
    let len;
    let data = new FlagStopResponse(null);
    // Deserialize message field [flag_stop]
    data.flag_stop = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'collision_avoidance/FlagStopResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '05b5eaa636771131c617bd4935b80111';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool flag_stop
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlagStopResponse(null);
    if (msg.flag_stop !== undefined) {
      resolved.flag_stop = msg.flag_stop;
    }
    else {
      resolved.flag_stop = false
    }

    return resolved;
    }
};

module.exports = {
  Request: FlagStopRequest,
  Response: FlagStopResponse,
  md5sum() { return '7b0aefe35bb755536b8814d5dcb2c19d'; },
  datatype() { return 'collision_avoidance/FlagStop'; }
};
