; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-msg)


;//! \htmlinclude GeoPose.msg.html

(cl:defclass <GeoPose> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type autopilot_bridge-msg:GeoPoint
    :initform (cl:make-instance 'autopilot_bridge-msg:GeoPoint))
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion)))
)

(cl:defclass GeoPose (<GeoPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GeoPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GeoPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-msg:<GeoPose> is deprecated: use autopilot_bridge-msg:GeoPose instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <GeoPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:position-val is deprecated.  Use autopilot_bridge-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <GeoPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:orientation-val is deprecated.  Use autopilot_bridge-msg:orientation instead.")
  (orientation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GeoPose>) ostream)
  "Serializes a message object of type '<GeoPose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GeoPose>) istream)
  "Deserializes a message object of type '<GeoPose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GeoPose>)))
  "Returns string type for a message object of type '<GeoPose>"
  "autopilot_bridge/GeoPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GeoPose)))
  "Returns string type for a message object of type 'GeoPose"
  "autopilot_bridge/GeoPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GeoPose>)))
  "Returns md5sum for a message object of type '<GeoPose>"
  "a73a1efca4f7df92e709875c74f317f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GeoPose)))
  "Returns md5sum for a message object of type 'GeoPose"
  "a73a1efca4f7df92e709875c74f317f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GeoPose>)))
  "Returns full string definition for message of type '<GeoPose>"
  (cl:format cl:nil "# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GeoPose)))
  "Returns full string definition for message of type 'GeoPose"
  (cl:format cl:nil "# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GeoPose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GeoPose>))
  "Converts a ROS message object to a list"
  (cl:list 'GeoPose
    (cl:cons ':position (position msg))
    (cl:cons ':orientation (orientation msg))
))
