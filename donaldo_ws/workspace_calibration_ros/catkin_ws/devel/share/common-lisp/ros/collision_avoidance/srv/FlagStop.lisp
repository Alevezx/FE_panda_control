; Auto-generated. Do not edit!


(cl:in-package collision_avoidance-srv)


;//! \htmlinclude FlagStop-request.msg.html

(cl:defclass <FlagStop-request> (roslisp-msg-protocol:ros-message)
  ((T_stop
    :reader T_stop
    :initarg :T_stop
    :type cl:float
    :initform 0.0)
   (q
    :reader q
    :initarg :q
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (q_p
    :reader q_p
    :initarg :q_p
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (q_pp
    :reader q_pp
    :initarg :q_pp
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass FlagStop-request (<FlagStop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlagStop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlagStop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collision_avoidance-srv:<FlagStop-request> is deprecated: use collision_avoidance-srv:FlagStop-request instead.")))

(cl:ensure-generic-function 'T_stop-val :lambda-list '(m))
(cl:defmethod T_stop-val ((m <FlagStop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-srv:T_stop-val is deprecated.  Use collision_avoidance-srv:T_stop instead.")
  (T_stop m))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <FlagStop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-srv:q-val is deprecated.  Use collision_avoidance-srv:q instead.")
  (q m))

(cl:ensure-generic-function 'q_p-val :lambda-list '(m))
(cl:defmethod q_p-val ((m <FlagStop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-srv:q_p-val is deprecated.  Use collision_avoidance-srv:q_p instead.")
  (q_p m))

(cl:ensure-generic-function 'q_pp-val :lambda-list '(m))
(cl:defmethod q_pp-val ((m <FlagStop-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-srv:q_pp-val is deprecated.  Use collision_avoidance-srv:q_pp instead.")
  (q_pp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlagStop-request>) ostream)
  "Serializes a message object of type '<FlagStop-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'T_stop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'q))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'q))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'q_p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'q_p))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'q_pp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'q_pp))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlagStop-request>) istream)
  "Deserializes a message object of type '<FlagStop-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'T_stop) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'q) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'q)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'q_p) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'q_p)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'q_pp) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'q_pp)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlagStop-request>)))
  "Returns string type for a service object of type '<FlagStop-request>"
  "collision_avoidance/FlagStopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlagStop-request)))
  "Returns string type for a service object of type 'FlagStop-request"
  "collision_avoidance/FlagStopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlagStop-request>)))
  "Returns md5sum for a message object of type '<FlagStop-request>"
  "7b0aefe35bb755536b8814d5dcb2c19d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlagStop-request)))
  "Returns md5sum for a message object of type 'FlagStop-request"
  "7b0aefe35bb755536b8814d5dcb2c19d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlagStop-request>)))
  "Returns full string definition for message of type '<FlagStop-request>"
  (cl:format cl:nil "float64 T_stop~%float64[] q~%float64[] q_p~%float64[] q_pp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlagStop-request)))
  "Returns full string definition for message of type 'FlagStop-request"
  (cl:format cl:nil "float64 T_stop~%float64[] q~%float64[] q_p~%float64[] q_pp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlagStop-request>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q_p) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q_pp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlagStop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FlagStop-request
    (cl:cons ':T_stop (T_stop msg))
    (cl:cons ':q (q msg))
    (cl:cons ':q_p (q_p msg))
    (cl:cons ':q_pp (q_pp msg))
))
;//! \htmlinclude FlagStop-response.msg.html

(cl:defclass <FlagStop-response> (roslisp-msg-protocol:ros-message)
  ((flag_stop
    :reader flag_stop
    :initarg :flag_stop
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FlagStop-response (<FlagStop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlagStop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlagStop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collision_avoidance-srv:<FlagStop-response> is deprecated: use collision_avoidance-srv:FlagStop-response instead.")))

(cl:ensure-generic-function 'flag_stop-val :lambda-list '(m))
(cl:defmethod flag_stop-val ((m <FlagStop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-srv:flag_stop-val is deprecated.  Use collision_avoidance-srv:flag_stop instead.")
  (flag_stop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlagStop-response>) ostream)
  "Serializes a message object of type '<FlagStop-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag_stop) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlagStop-response>) istream)
  "Deserializes a message object of type '<FlagStop-response>"
    (cl:setf (cl:slot-value msg 'flag_stop) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlagStop-response>)))
  "Returns string type for a service object of type '<FlagStop-response>"
  "collision_avoidance/FlagStopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlagStop-response)))
  "Returns string type for a service object of type 'FlagStop-response"
  "collision_avoidance/FlagStopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlagStop-response>)))
  "Returns md5sum for a message object of type '<FlagStop-response>"
  "7b0aefe35bb755536b8814d5dcb2c19d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlagStop-response)))
  "Returns md5sum for a message object of type 'FlagStop-response"
  "7b0aefe35bb755536b8814d5dcb2c19d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlagStop-response>)))
  "Returns full string definition for message of type '<FlagStop-response>"
  (cl:format cl:nil "bool flag_stop~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlagStop-response)))
  "Returns full string definition for message of type 'FlagStop-response"
  (cl:format cl:nil "bool flag_stop~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlagStop-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlagStop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FlagStop-response
    (cl:cons ':flag_stop (flag_stop msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FlagStop)))
  'FlagStop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FlagStop)))
  'FlagStop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlagStop)))
  "Returns string type for a service object of type '<FlagStop>"
  "collision_avoidance/FlagStop")