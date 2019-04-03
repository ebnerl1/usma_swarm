; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-msg)


;//! \htmlinclude Heartbeat.msg.html

(cl:defclass <Heartbeat> (roslisp-msg-protocol:ros-message)
  ((counter
    :reader counter
    :initarg :counter
    :type cl:integer
    :initform 0))
)

(cl:defclass Heartbeat (<Heartbeat>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Heartbeat>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Heartbeat)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-msg:<Heartbeat> is deprecated: use autopilot_bridge-msg:Heartbeat instead.")))

(cl:ensure-generic-function 'counter-val :lambda-list '(m))
(cl:defmethod counter-val ((m <Heartbeat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:counter-val is deprecated.  Use autopilot_bridge-msg:counter instead.")
  (counter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Heartbeat>) ostream)
  "Serializes a message object of type '<Heartbeat>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'counter)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Heartbeat>) istream)
  "Deserializes a message object of type '<Heartbeat>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'counter)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Heartbeat>)))
  "Returns string type for a message object of type '<Heartbeat>"
  "autopilot_bridge/Heartbeat")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Heartbeat)))
  "Returns string type for a message object of type 'Heartbeat"
  "autopilot_bridge/Heartbeat")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Heartbeat>)))
  "Returns md5sum for a message object of type '<Heartbeat>"
  "edfa4d08cb5e6d4c5fd223930476506d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Heartbeat)))
  "Returns md5sum for a message object of type 'Heartbeat"
  "edfa4d08cb5e6d4c5fd223930476506d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Heartbeat>)))
  "Returns full string definition for message of type '<Heartbeat>"
  (cl:format cl:nil "uint32 counter~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Heartbeat)))
  "Returns full string definition for message of type 'Heartbeat"
  (cl:format cl:nil "uint32 counter~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Heartbeat>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Heartbeat>))
  "Converts a ROS message object to a list"
  (cl:list 'Heartbeat
    (cl:cons ':counter (counter msg))
))
