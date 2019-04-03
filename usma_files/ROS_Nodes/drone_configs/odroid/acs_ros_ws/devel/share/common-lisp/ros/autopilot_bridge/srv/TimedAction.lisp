; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude TimedAction-request.msg.html

(cl:defclass <TimedAction-request> (roslisp-msg-protocol:ros-message)
  ((timeout
    :reader timeout
    :initarg :timeout
    :type cl:float
    :initform 0.0))
)

(cl:defclass TimedAction-request (<TimedAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TimedAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TimedAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<TimedAction-request> is deprecated: use autopilot_bridge-srv:TimedAction-request instead.")))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <TimedAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:timeout-val is deprecated.  Use autopilot_bridge-srv:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TimedAction-request>) ostream)
  "Serializes a message object of type '<TimedAction-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'timeout))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TimedAction-request>) istream)
  "Deserializes a message object of type '<TimedAction-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeout) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TimedAction-request>)))
  "Returns string type for a service object of type '<TimedAction-request>"
  "autopilot_bridge/TimedActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TimedAction-request)))
  "Returns string type for a service object of type 'TimedAction-request"
  "autopilot_bridge/TimedActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TimedAction-request>)))
  "Returns md5sum for a message object of type '<TimedAction-request>"
  "64f472a0a4b1503ba77d41b239c07c42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TimedAction-request)))
  "Returns md5sum for a message object of type 'TimedAction-request"
  "64f472a0a4b1503ba77d41b239c07c42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TimedAction-request>)))
  "Returns full string definition for message of type '<TimedAction-request>"
  (cl:format cl:nil "float32 timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TimedAction-request)))
  "Returns full string definition for message of type 'TimedAction-request"
  (cl:format cl:nil "float32 timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TimedAction-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TimedAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TimedAction-request
    (cl:cons ':timeout (timeout msg))
))
;//! \htmlinclude TimedAction-response.msg.html

(cl:defclass <TimedAction-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TimedAction-response (<TimedAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TimedAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TimedAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<TimedAction-response> is deprecated: use autopilot_bridge-srv:TimedAction-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <TimedAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TimedAction-response>) ostream)
  "Serializes a message object of type '<TimedAction-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TimedAction-response>) istream)
  "Deserializes a message object of type '<TimedAction-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TimedAction-response>)))
  "Returns string type for a service object of type '<TimedAction-response>"
  "autopilot_bridge/TimedActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TimedAction-response)))
  "Returns string type for a service object of type 'TimedAction-response"
  "autopilot_bridge/TimedActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TimedAction-response>)))
  "Returns md5sum for a message object of type '<TimedAction-response>"
  "64f472a0a4b1503ba77d41b239c07c42")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TimedAction-response)))
  "Returns md5sum for a message object of type 'TimedAction-response"
  "64f472a0a4b1503ba77d41b239c07c42")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TimedAction-response>)))
  "Returns full string definition for message of type '<TimedAction-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TimedAction-response)))
  "Returns full string definition for message of type 'TimedAction-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TimedAction-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TimedAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TimedAction-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TimedAction)))
  'TimedAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TimedAction)))
  'TimedAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TimedAction)))
  "Returns string type for a service object of type '<TimedAction>"
  "autopilot_bridge/TimedAction")