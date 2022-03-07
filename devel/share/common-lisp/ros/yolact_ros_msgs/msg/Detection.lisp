; Auto-generated. Do not edit!


(cl:in-package yolact_ros_msgs-msg)


;//! \htmlinclude Detection.msg.html

(cl:defclass <Detection> (roslisp-msg-protocol:ros-message)
  ((class_name
    :reader class_name
    :initarg :class_name
    :type cl:string
    :initform "")
   (score
    :reader score
    :initarg :score
    :type cl:float
    :initform 0.0)
   (box
    :reader box
    :initarg :box
    :type yolact_ros_msgs-msg:Box
    :initform (cl:make-instance 'yolact_ros_msgs-msg:Box))
   (mask
    :reader mask
    :initarg :mask
    :type yolact_ros_msgs-msg:Mask
    :initform (cl:make-instance 'yolact_ros_msgs-msg:Mask)))
)

(cl:defclass Detection (<Detection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Detection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Detection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yolact_ros_msgs-msg:<Detection> is deprecated: use yolact_ros_msgs-msg:Detection instead.")))

(cl:ensure-generic-function 'class_name-val :lambda-list '(m))
(cl:defmethod class_name-val ((m <Detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolact_ros_msgs-msg:class_name-val is deprecated.  Use yolact_ros_msgs-msg:class_name instead.")
  (class_name m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <Detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolact_ros_msgs-msg:score-val is deprecated.  Use yolact_ros_msgs-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'box-val :lambda-list '(m))
(cl:defmethod box-val ((m <Detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolact_ros_msgs-msg:box-val is deprecated.  Use yolact_ros_msgs-msg:box instead.")
  (box m))

(cl:ensure-generic-function 'mask-val :lambda-list '(m))
(cl:defmethod mask-val ((m <Detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolact_ros_msgs-msg:mask-val is deprecated.  Use yolact_ros_msgs-msg:mask instead.")
  (mask m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Detection>) ostream)
  "Serializes a message object of type '<Detection>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'class_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'class_name))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'box) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'mask) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Detection>) istream)
  "Deserializes a message object of type '<Detection>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'class_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'class_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'score) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'box) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'mask) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Detection>)))
  "Returns string type for a message object of type '<Detection>"
  "yolact_ros_msgs/Detection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Detection)))
  "Returns string type for a message object of type 'Detection"
  "yolact_ros_msgs/Detection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Detection>)))
  "Returns md5sum for a message object of type '<Detection>"
  "b9e8252cb1b190158c1553f6547a1db7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Detection)))
  "Returns md5sum for a message object of type 'Detection"
  "b9e8252cb1b190158c1553f6547a1db7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Detection>)))
  "Returns full string definition for message of type '<Detection>"
  (cl:format cl:nil "string class_name~%float64 score~%Box box~%Mask mask~%~%================================================================================~%MSG: yolact_ros_msgs/Box~%int32 x1~%int32 y1~%int32 x2~%int32 y2~%~%================================================================================~%MSG: yolact_ros_msgs/Mask~%int32 width~%int32 height~%uint8[] mask # Mask encoded as bitset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Detection)))
  "Returns full string definition for message of type 'Detection"
  (cl:format cl:nil "string class_name~%float64 score~%Box box~%Mask mask~%~%================================================================================~%MSG: yolact_ros_msgs/Box~%int32 x1~%int32 y1~%int32 x2~%int32 y2~%~%================================================================================~%MSG: yolact_ros_msgs/Mask~%int32 width~%int32 height~%uint8[] mask # Mask encoded as bitset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Detection>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'class_name))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'box))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'mask))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Detection>))
  "Converts a ROS message object to a list"
  (cl:list 'Detection
    (cl:cons ':class_name (class_name msg))
    (cl:cons ':score (score msg))
    (cl:cons ':box (box msg))
    (cl:cons ':mask (mask msg))
))
