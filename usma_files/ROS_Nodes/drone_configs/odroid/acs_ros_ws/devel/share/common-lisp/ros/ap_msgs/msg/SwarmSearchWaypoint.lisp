; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude SwarmSearchWaypoint.msg.html

(cl:defclass <SwarmSearchWaypoint> (roslisp-msg-protocol:ros-message)
  ((recipientvehicle_id
    :reader recipientvehicle_id
    :initarg :recipientvehicle_id
    :type cl:fixnum
    :initform 0)
   (searchCell_x
    :reader searchCell_x
    :initarg :searchCell_x
    :type cl:fixnum
    :initform 0)
   (searchCell_y
    :reader searchCell_y
    :initarg :searchCell_y
    :type cl:fixnum
    :initform 0)
   (waypoint
    :reader waypoint
    :initarg :waypoint
    :type autopilot_bridge-msg:LLA
    :initform (cl:make-instance 'autopilot_bridge-msg:LLA)))
)

(cl:defclass SwarmSearchWaypoint (<SwarmSearchWaypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwarmSearchWaypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwarmSearchWaypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<SwarmSearchWaypoint> is deprecated: use ap_msgs-msg:SwarmSearchWaypoint instead.")))

(cl:ensure-generic-function 'recipientvehicle_id-val :lambda-list '(m))
(cl:defmethod recipientvehicle_id-val ((m <SwarmSearchWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:recipientvehicle_id-val is deprecated.  Use ap_msgs-msg:recipientvehicle_id instead.")
  (recipientvehicle_id m))

(cl:ensure-generic-function 'searchCell_x-val :lambda-list '(m))
(cl:defmethod searchCell_x-val ((m <SwarmSearchWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:searchCell_x-val is deprecated.  Use ap_msgs-msg:searchCell_x instead.")
  (searchCell_x m))

(cl:ensure-generic-function 'searchCell_y-val :lambda-list '(m))
(cl:defmethod searchCell_y-val ((m <SwarmSearchWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:searchCell_y-val is deprecated.  Use ap_msgs-msg:searchCell_y instead.")
  (searchCell_y m))

(cl:ensure-generic-function 'waypoint-val :lambda-list '(m))
(cl:defmethod waypoint-val ((m <SwarmSearchWaypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:waypoint-val is deprecated.  Use ap_msgs-msg:waypoint instead.")
  (waypoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwarmSearchWaypoint>) ostream)
  "Serializes a message object of type '<SwarmSearchWaypoint>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'recipientvehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'searchCell_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'searchCell_y)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'waypoint) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwarmSearchWaypoint>) istream)
  "Deserializes a message object of type '<SwarmSearchWaypoint>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'recipientvehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'searchCell_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'searchCell_y)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'waypoint) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwarmSearchWaypoint>)))
  "Returns string type for a message object of type '<SwarmSearchWaypoint>"
  "ap_msgs/SwarmSearchWaypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwarmSearchWaypoint)))
  "Returns string type for a message object of type 'SwarmSearchWaypoint"
  "ap_msgs/SwarmSearchWaypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwarmSearchWaypoint>)))
  "Returns md5sum for a message object of type '<SwarmSearchWaypoint>"
  "e251703b3a9805e75514677c841efc23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwarmSearchWaypoint)))
  "Returns md5sum for a message object of type 'SwarmSearchWaypoint"
  "e251703b3a9805e75514677c841efc23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwarmSearchWaypoint>)))
  "Returns full string definition for message of type '<SwarmSearchWaypoint>"
  (cl:format cl:nil "uint8 recipientvehicle_id~%uint8 searchCell_x~%uint8 searchCell_y~%autopilot_bridge/LLA waypoint~%~%================================================================================~%MSG: autopilot_bridge/LLA~%float64 lat~%float64 lon~%float64 alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwarmSearchWaypoint)))
  "Returns full string definition for message of type 'SwarmSearchWaypoint"
  (cl:format cl:nil "uint8 recipientvehicle_id~%uint8 searchCell_x~%uint8 searchCell_y~%autopilot_bridge/LLA waypoint~%~%================================================================================~%MSG: autopilot_bridge/LLA~%float64 lat~%float64 lon~%float64 alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwarmSearchWaypoint>))
  (cl:+ 0
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'waypoint))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwarmSearchWaypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'SwarmSearchWaypoint
    (cl:cons ':recipientvehicle_id (recipientvehicle_id msg))
    (cl:cons ':searchCell_x (searchCell_x msg))
    (cl:cons ':searchCell_y (searchCell_y msg))
    (cl:cons ':waypoint (waypoint msg))
))
