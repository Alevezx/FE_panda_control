; Auto-generated. Do not edit!


(cl:in-package collision_avoidance-srv)


;//! \htmlinclude CalcStopDuration-request.msg.html

(cl:defclass <CalcStopDuration-request> (roslisp-msg-protocol:ros-message)
  ((q
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

(cl:defclass CalcStopDuration-request (<CalcStopDuration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalcStopDuration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalcStopDuration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collision_avoidance-srv:<CalcStopDuration-request> is deprecated: use collision_avoidance-srv:CalcStopDuration-request instead.")))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <CalcStopDuration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-srv:q-val is deprecated.  Use collision_avoidance-srv:q instead.")
  (q m))

(cl:ensure-generic-function 'q_p-val :lambda-list '(m))
(cl:defmethod q_p-val ((m <CalcStopDuration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-srv:q_p-val is deprecated.  Use collision_avoidance-srv:q_p instead.")
  (q_p m))

(cl:ensure-generic-function 'q_pp-val :lambda-list '(m))
(cl:defmethod q_pp-val ((m <CalcStopDuration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-srv:q_pp-val is deprecated.  Use collision_avoidance-srv:q_pp instead.")
  (q_pp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalcStopDuration-request>) ostream)
  "Serializes a message object of type '<CalcStopDuration-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalcStopDuration-request>) istream)
  "Deserializes a message object of type '<CalcStopDuration-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalcStopDuration-request>)))
  "Returns string type for a service object of type '<CalcStopDuration-request>"
  "collision_avoidance/CalcStopDurationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalcStopDuration-request)))
  "Returns string type for a service object of type 'CalcStopDuration-request"
  "collision_avoidance/CalcStopDurationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalcStopDuration-request>)))
  "Returns md5sum for a message object of type '<CalcStopDuration-request>"
  "a8531c49262635abe6abcd3257d5d874")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalcStopDuration-request)))
  "Returns md5sum for a message object of type 'CalcStopDuration-request"
  "a8531c49262635abe6abcd3257d5d874")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalcStopDuration-request>)))
  "Returns full string definition for message of type '<CalcStopDuration-request>"
  (cl:format cl:nil "float64[] q~%float64[] q_p~%float64[] q_pp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalcStopDuration-request)))
  "Returns full string definition for message of type 'CalcStopDuration-request"
  (cl:format cl:nil "float64[] q~%float64[] q_p~%float64[] q_pp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalcStopDuration-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q_p) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'q_pp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalcStopDuration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CalcStopDuration-request
    (cl:cons ':q (q msg))
    (cl:cons ':q_p (q_p msg))
    (cl:cons ':q_pp (q_pp msg))
))
;//! \htmlinclude CalcStopDuration-response.msg.html

(cl:defclass <CalcStopDuration-response> (roslisp-msg-protocol:ros-message)
  ((stop_duration
    :reader stop_duration
    :initarg :stop_duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass CalcStopDuration-response (<CalcStopDuration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CalcStopDuration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CalcStopDuration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name collision_avoidance-srv:<CalcStopDuration-response> is deprecated: use collision_avoidance-srv:CalcStopDuration-response instead.")))

(cl:ensure-generic-function 'stop_duration-val :lambda-list '(m))
(cl:defmethod stop_duration-val ((m <CalcStopDuration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader collision_avoidance-srv:stop_duration-val is deprecated.  Use collision_avoidance-srv:stop_duration instead.")
  (stop_duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CalcStopDuration-response>) ostream)
  "Serializes a message object of type '<CalcStopDuration-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'stop_duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CalcStopDuration-response>) istream)
  "Deserializes a message object of type '<CalcStopDuration-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stop_duration) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CalcStopDuration-response>)))
  "Returns string type for a service object of type '<CalcStopDuration-response>"
  "collision_avoidance/CalcStopDurationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalcStopDuration-response)))
  "Returns string type for a service object of type 'CalcStopDuration-response"
  "collision_avoidance/CalcStopDurationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CalcStopDuration-response>)))
  "Returns md5sum for a message object of type '<CalcStopDuration-response>"
  "a8531c49262635abe6abcd3257d5d874")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CalcStopDuration-response)))
  "Returns md5sum for a message object of type 'CalcStopDuration-response"
  "a8531c49262635abe6abcd3257d5d874")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CalcStopDuration-response>)))
  "Returns full string definition for message of type '<CalcStopDuration-response>"
  (cl:format cl:nil "float64 stop_duration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CalcStopDuration-response)))
  "Returns full string definition for message of type 'CalcStopDuration-response"
  (cl:format cl:nil "float64 stop_duration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CalcStopDuration-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CalcStopDuration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CalcStopDuration-response
    (cl:cons ':stop_duration (stop_duration msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CalcStopDuration)))
  'CalcStopDuration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CalcStopDuration)))
  'CalcStopDuration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CalcStopDuration)))
  "Returns string type for a service object of type '<CalcStopDuration>"
  "collision_avoidance/CalcStopDuration")