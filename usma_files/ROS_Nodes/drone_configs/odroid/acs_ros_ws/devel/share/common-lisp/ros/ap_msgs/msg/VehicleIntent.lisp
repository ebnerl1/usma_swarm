; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude VehicleIntent.msg.html

(cl:defclass <VehicleIntent> (roslisp-msg-protocol:ros-message)
  ((vehicle_id
    :reader vehicle_id
    :initarg :vehicle_id
    :type cl:fixnum
    :initform 0)
   (swarm_behavior
    :reader swarm_behavior
    :initarg :swarm_behavior
    :type cl:fixnum
    :initform 0)
   (loc
    :reader loc
    :initarg :loc
    :type autopilot_bridge-msg:LLA
    :initform (cl:make-instance 'autopilot_bridge-msg:LLA)))
)

(cl:defclass VehicleIntent (<VehicleIntent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehicleIntent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehicleIntent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<VehicleIntent> is deprecated: use ap_msgs-msg:VehicleIntent instead.")))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <VehicleIntent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:vehicle_id-val is deprecated.  Use ap_msgs-msg:vehicle_id instead.")
  (vehicle_id m))

(cl:ensure-generic-function 'swarm_behavior-val :lambda-list '(m))
(cl:defmethod swarm_behavior-val ((m <VehicleIntent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:swarm_behavior-val is deprecated.  Use ap_msgs-msg:swarm_behavior instead.")
  (swarm_behavior m))

(cl:ensure-generic-function 'loc-val :lambda-list '(m))
(cl:defmethod loc-val ((m <VehicleIntent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:loc-val is deprecated.  Use ap_msgs-msg:loc instead.")
  (loc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehicleIntent>) ostream)
  "Serializes a message object of type '<VehicleIntent>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_behavior)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'loc) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehicleIntent>) istream)
  "Deserializes a message object of type '<VehicleIntent>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_behavior)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'loc) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehicleIntent>)))
  "Returns string type for a message object of type '<VehicleIntent>"
  "ap_msgs/VehicleIntent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehicleIntent)))
  "Returns string type for a message object of type 'VehicleIntent"
  "ap_msgs/VehicleIntent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehicleIntent>)))
  "Returns md5sum for a message object of type '<VehicleIntent>"
  "1baad67ae12e46c06ebeaa3de44f6e6e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehicleIntent)))
  "Returns md5sum for a message object of type 'VehicleIntent"
  "1baad67ae12e46c06ebeaa3de44f6e6e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehicleIntent>)))
  "Returns full string definition for message of type '<VehicleIntent>"
  (cl:format cl:nil "uint8 vehicle_id~%uint8 swarm_behavior~%autopilot_bridge/LLA loc~%~%================================================================================~%MSG: autopilot_bridge/LLA~%float64 lat~%float64 lon~%float64 alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehicleIntent)))
  "Returns full string definition for message of type 'VehicleIntent"
  (cl:format cl:nil "uint8 vehicle_id~%uint8 swarm_behavior~%autopilot_bridge/LLA loc~%~%================================================================================~%MSG: autopilot_bridge/LLA~%float64 lat~%float64 lon~%float64 alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehicleIntent>))
  (cl:+ 0
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'loc))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehicleIntent>))
  "Converts a ROS message object to a list"
  (cl:list 'VehicleIntent
    (cl:cons ':vehicle_id (vehicle_id msg))
    (cl:cons ':swarm_behavior (swarm_behavior msg))
    (cl:cons ':loc (loc msg))
))
