; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude ParamSetList-request.msg.html

(cl:defclass <ParamSetList-request> (roslisp-msg-protocol:ros-message)
  ((param
    :reader param
    :initarg :param
    :type (cl:vector autopilot_bridge-msg:ParamPair)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:ParamPair :initial-element (cl:make-instance 'autopilot_bridge-msg:ParamPair))))
)

(cl:defclass ParamSetList-request (<ParamSetList-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParamSetList-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParamSetList-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<ParamSetList-request> is deprecated: use autopilot_bridge-srv:ParamSetList-request instead.")))

(cl:ensure-generic-function 'param-val :lambda-list '(m))
(cl:defmethod param-val ((m <ParamSetList-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:param-val is deprecated.  Use autopilot_bridge-srv:param instead.")
  (param m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParamSetList-request>) ostream)
  "Serializes a message object of type '<ParamSetList-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'param))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParamSetList-request>) istream)
  "Deserializes a message object of type '<ParamSetList-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParamSetList-request>)))
  "Returns string type for a service object of type '<ParamSetList-request>"
  "autopilot_bridge/ParamSetListRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamSetList-request)))
  "Returns string type for a service object of type 'ParamSetList-request"
  "autopilot_bridge/ParamSetListRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParamSetList-request>)))
  "Returns md5sum for a message object of type '<ParamSetList-request>"
  "7c20195a6434b00060c5d34397ef8702")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParamSetList-request)))
  "Returns md5sum for a message object of type 'ParamSetList-request"
  "7c20195a6434b00060c5d34397ef8702")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParamSetList-request>)))
  "Returns full string definition for message of type '<ParamSetList-request>"
  (cl:format cl:nil "ParamPair[] param~%~%================================================================================~%MSG: autopilot_bridge/ParamPair~%string name~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParamSetList-request)))
  "Returns full string definition for message of type 'ParamSetList-request"
  (cl:format cl:nil "ParamPair[] param~%~%================================================================================~%MSG: autopilot_bridge/ParamPair~%string name~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParamSetList-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'param) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParamSetList-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ParamSetList-request
    (cl:cons ':param (param msg))
))
;//! \htmlinclude ParamSetList-response.msg.html

(cl:defclass <ParamSetList-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ParamSetList-response (<ParamSetList-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ParamSetList-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ParamSetList-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<ParamSetList-response> is deprecated: use autopilot_bridge-srv:ParamSetList-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <ParamSetList-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ParamSetList-response>) ostream)
  "Serializes a message object of type '<ParamSetList-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ParamSetList-response>) istream)
  "Deserializes a message object of type '<ParamSetList-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ParamSetList-response>)))
  "Returns string type for a service object of type '<ParamSetList-response>"
  "autopilot_bridge/ParamSetListResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamSetList-response)))
  "Returns string type for a service object of type 'ParamSetList-response"
  "autopilot_bridge/ParamSetListResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ParamSetList-response>)))
  "Returns md5sum for a message object of type '<ParamSetList-response>"
  "7c20195a6434b00060c5d34397ef8702")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ParamSetList-response)))
  "Returns md5sum for a message object of type 'ParamSetList-response"
  "7c20195a6434b00060c5d34397ef8702")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ParamSetList-response>)))
  "Returns full string definition for message of type '<ParamSetList-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ParamSetList-response)))
  "Returns full string definition for message of type 'ParamSetList-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ParamSetList-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ParamSetList-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ParamSetList-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ParamSetList)))
  'ParamSetList-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ParamSetList)))
  'ParamSetList-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ParamSetList)))
  "Returns string type for a service object of type '<ParamSetList>"
  "autopilot_bridge/ParamSetList")