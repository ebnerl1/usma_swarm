; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-msg)


;//! \htmlinclude Takeoff.msg.html

(cl:defclass <Takeoff> (roslisp-msg-protocol:ros-message)
  ((takeoff_alt
    :reader takeoff_alt
    :initarg :takeoff_alt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Takeoff (<Takeoff>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Takeoff>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Takeoff)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-msg:<Takeoff> is deprecated: use autopilot_bridge-msg:Takeoff instead.")))

(cl:ensure-generic-function 'takeoff_alt-val :lambda-list '(m))
(cl:defmethod takeoff_alt-val ((m <Takeoff>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:takeoff_alt-val is deprecated.  Use autopilot_bridge-msg:takeoff_alt instead.")
  (takeoff_alt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Takeoff>) ostream)
  "Serializes a message object of type '<Takeoff>"
  (cl:let* ((signed (cl:slot-value msg 'takeoff_alt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Takeoff>) istream)
  "Deserializes a message object of type '<Takeoff>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'takeoff_alt) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Takeoff>)))
  "Returns string type for a message object of type '<Takeoff>"
  "autopilot_bridge/Takeoff")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Takeoff)))
  "Returns string type for a message object of type 'Takeoff"
  "autopilot_bridge/Takeoff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Takeoff>)))
  "Returns md5sum for a message object of type '<Takeoff>"
  "0e2e05d2ce937de092e57b75b1c2ea72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Takeoff)))
  "Returns md5sum for a message object of type 'Takeoff"
  "0e2e05d2ce937de092e57b75b1c2ea72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Takeoff>)))
  "Returns full string definition for message of type '<Takeoff>"
  (cl:format cl:nil "int16 takeoff_alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Takeoff)))
  "Returns full string definition for message of type 'Takeoff"
  (cl:format cl:nil "int16 takeoff_alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Takeoff>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Takeoff>))
  "Converts a ROS message object to a list"
  (cl:list 'Takeoff
    (cl:cons ':takeoff_alt (takeoff_alt msg))
))
