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

class CalcStopDurationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.q = null;
      this.q_p = null;
      this.q_pp = null;
    }
    else {
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
    // Serializes a message object of type CalcStopDurationRequest
    // Serialize message field [q]
    bufferOffset = _arraySerializer.float64(obj.q, buffer, bufferOffset, null);
    // Serialize message field [q_p]
    bufferOffset = _arraySerializer.float64(obj.q_p, buffer, bufferOffset, null);
    // Serialize message field [q_pp]
    bufferOffset = _arraySerializer.float64(obj.q_pp, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalcStopDurationRequest
    let len;
    let data = new CalcStopDurationRequest(null);
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
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'collision_avoidance/CalcStopDurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6fd5787f799b6515a9da80500d9f73fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new CalcStopDurationRequest(null);
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

class CalcStopDurationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stop_duration = null;
    }
    else {
      if (initObj.hasOwnProperty('stop_duration')) {
        this.stop_duration = initObj.stop_duration
      }
      else {
        this.stop_duration = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalcStopDurationResponse
    // Serialize message field [stop_duration]
    bufferOffset = _serializer.float64(obj.stop_duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalcStopDurationResponse
    let len;
    let data = new CalcStopDurationResponse(null);
    // Deserialize message field [stop_duration]
    data.stop_duration = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'collision_avoidance/CalcStopDurationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '596df0feed32a80b1c46d99baf544050';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 stop_duration
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalcStopDurationResponse(null);
    if (msg.stop_duration !== undefined) {
      resolved.stop_duration = msg.stop_duration;
    }
    else {
      resolved.stop_duration = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: CalcStopDurationRequest,
  Response: CalcStopDurationResponse,
  md5sum() { return 'a8531c49262635abe6abcd3257d5d874'; },
  datatype() { return 'collision_avoidance/CalcStopDuration'; }
};
