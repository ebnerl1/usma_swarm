; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude ParamGetList-request.msg.html

(cl:defclass <ParamGetList-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ParamGetList-request (<ParamGetList-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParamGetList-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParamGetList-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<ParamGetList-request> is deprecated: use autopilot_bridge-srv:ParamGetList-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ParamGetList-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:name-val is deprecated.  Use autopilot_bridge-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParamGetList-request>) ostream)
  "Serializes a message object of type '<ParamGetList-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParamGetList-request>) istream)
  "Deserializes a message object of type '<ParamGetList-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'name) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'name)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParamGetList-request>)))
  "Returns string type for a service object of type '<ParamGetList-request>"
  "autopilot_bridge/ParamGetListRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamGetList-request)))
  "Returns string type for a service object of type 'ParamGetList-request"
  "autopilot_bridge/ParamGetListRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParamGetList-request>)))
  "Returns md5sum for a message object of type '<ParamGetList-request>"
  "e2cef5924ec02ac5c653b10adb157290")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParamGetList-request)))
  "Returns md5sum for a message object of type 'ParamGetList-request"
  "e2cef5924ec02ac5c653b10adb157290")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParamGetList-request>)))
  "Returns full string definition for message of type '<ParamGetList-request>"
  (cl:format cl:nil "string[] name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParamGetList-request)))
  "Returns full string definition for message of type 'ParamGetList-request"
  (cl:format cl:nil "string[] name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParamGetList-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'name) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParamGetList-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ParamGetList-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude ParamGetList-response.msg.html

(cl:defclass <ParamGetList-response> (roslisp-msg-protocol:ros-message)
  ((param
    :reader param
    :initarg :param
    :type (cl:vector autopilot_bridge-msg:ParamPair)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:ParamPair :initial-element (cl:make-instance 'autopilot_bridge-msg:ParamPair))))
)

(cl:defclass ParamGetList-response (<ParamGetList-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParamGetList-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParamGetList-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<ParamGetList-response> is deprecated: use autopilot_bridge-srv:ParamGetList-response instead.")))

(cl:ensure-generic-function 'param-val :lambda-list '(m))
(cl:defmethod param-val ((m <ParamGetList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:param-val is deprecated.  Use autopilot_bridge-srv:param instead.")
  (param m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParamGetList-response>) ostream)
  "Serializes a message object of type '<ParamGetList-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'param))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParamGetList-response>) istream)
  "Deserializes a message object of type '<ParamGetList-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'param) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'param)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'autopilot_bridge-msg:ParamPair))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParamGetList-response>)))
  "Returns string type for a service object of type '<ParamGetList-response>"
  "autopilot_bridge/ParamGetListResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamGetList-response)))
  "Returns string type for a service object of type 'ParamGetList-response"
  "autopilot_bridge/ParamGetListResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParamGetList-response>)))
  "Returns md5sum for a message object of type '<ParamGetList-response>"
  "e2cef5924ec02ac5c653b10adb157290")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParamGetList-response)))
  "Returns md5sum for a message object of type 'ParamGetList-response"
  "e2cef5924ec02ac5c653b10adb157290")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParamGetList-response>)))
  "Returns full string definition for message of type '<ParamGetList-response>"
  (cl:format cl:nil "ParamPair[] param~%~%~%================================================================================~%MSG: autopilot_bridge/ParamPair~%string name~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParamGetList-response)))
  "Returns full string definition for message of type 'ParamGetList-response"
  (cl:format cl:nil "ParamPair[] param~%~%~%================================================================================~%MSG: autopilot_bridge/ParamPair~%string name~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParamGetList-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'param) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParamGetList-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ParamGetList-response
    (cl:cons ':param (param msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ParamGetList)))
  'ParamGetList-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ParamGetList)))
  'ParamGetList-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamGetList)))
  "Returns string type for a service object of type '<ParamGetList>"
  "autopilot_bridge/ParamGetList")