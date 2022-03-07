// Auto-generated. Do not edit!

// (in-package yolact_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Box = require('./Box.js');
let Mask = require('./Mask.js');

//-----------------------------------------------------------

class Detection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.class_name = null;
      this.score = null;
      this.box = null;
      this.mask = null;
    }
    else {
      if (initObj.hasOwnProperty('class_name')) {
        this.class_name = initObj.class_name
      }
      else {
        this.class_name = '';
      }
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0.0;
      }
      if (initObj.hasOwnProperty('box')) {
        this.box = initObj.box
      }
      else {
        this.box = new Box();
      }
      if (initObj.hasOwnProperty('mask')) {
        this.mask = initObj.mask
      }
      else {
        this.mask = new Mask();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Detection
    // Serialize message field [class_name]
    bufferOffset = _serializer.string(obj.class_name, buffer, bufferOffset);
    // Serialize message field [score]
    bufferOffset = _serializer.float64(obj.score, buffer, bufferOffset);
    // Serialize message field [box]
    bufferOffset = Box.serialize(obj.box, buffer, bufferOffset);
    // Serialize message field [mask]
    bufferOffset = Mask.serialize(obj.mask, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Detection
    let len;
    let data = new Detection(null);
    // Deserialize message field [class_name]
    data.class_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [score]
    data.score = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [box]
    data.box = Box.deserialize(buffer, bufferOffset);
    // Deserialize message field [mask]
    data.mask = Mask.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.class_name.length;
    length += Mask.getMessageSize(object.mask);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yolact_ros_msgs/Detection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b9e8252cb1b190158c1553f6547a1db7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string class_name
    float64 score
    Box box
    Mask mask
    
    ================================================================================
    MSG: yolact_ros_msgs/Box
    int32 x1
    int32 y1
    int32 x2
    int32 y2
    
    ================================================================================
    MSG: yolact_ros_msgs/Mask
    int32 width
    int32 height
    uint8[] mask # Mask encoded as bitset
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Detection(null);
    if (msg.class_name !== undefined) {
      resolved.class_name = msg.class_name;
    }
    else {
      resolved.class_name = ''
    }

    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0.0
    }

    if (msg.box !== undefined) {
      resolved.box = Box.Resolve(msg.box)
    }
    else {
      resolved.box = new Box()
    }

    if (msg.mask !== undefined) {
      resolved.mask = Mask.Resolve(msg.mask)
    }
    else {
      resolved.mask = new Mask()
    }

    return resolved;
    }
};

module.exports = Detection;
