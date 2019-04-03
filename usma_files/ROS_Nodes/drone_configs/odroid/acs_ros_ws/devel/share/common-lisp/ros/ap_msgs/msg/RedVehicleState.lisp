; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude RedVehicleState.msg.html

(cl:defclass <RedVehicleState> (roslisp-msg-protocol:ros-message)
  ((vehicle_id
    :reader vehicle_id
    :initarg :vehicle_id
    :type cl:fixnum
    :initform 0)
   (game_status
    :reader game_status
    :initarg :game_status
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type autopilot_bridge-msg:Geodometry
    :initform (cl:make-instance 'autopilot_bridge-msg:Geodometry)))
)

(cl:defclass RedVehicleState (<RedVehicleState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RedVehicleState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RedVehicleState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<RedVehicleState> is deprecated: use ap_msgs-msg:RedVehicleState instead.")))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <RedVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:vehicle_id-val is deprecated.  Use ap_msgs-msg:vehicle_id instead.")
  (vehicle_id m))

(cl:ensure-generic-function 'game_status-val :lambda-list '(m))
(cl:defmethod game_status-val ((m <RedVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:game_status-val is deprecated.  Use ap_msgs-msg:game_status instead.")
  (game_status m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <RedVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:state-val is deprecated.  Use ap_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RedVehicleState>) ostream)
  "Serializes a message object of type '<RedVehicleState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'game_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'game_status)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RedVehicleState>) istream)
  "Deserializes a message object of type '<RedVehicleState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'game_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'game_status)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RedVehicleState>)))
  "Returns string type for a message object of type '<RedVehicleState>"
  "ap_msgs/RedVehicleState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RedVehicleState)))
  "Returns string type for a message object of type 'RedVehicleState"
  "ap_msgs/RedVehicleState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RedVehicleState>)))
  "Returns md5sum for a message object of type '<RedVehicleState>"
  "1abd09f43bdba46b4c5660ac605c04a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RedVehicleState)))
  "Returns md5sum for a message object of type 'RedVehicleState"
  "1abd09f43bdba46b4c5660ac605c04a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RedVehicleState>)))
  "Returns full string definition for message of type '<RedVehicleState>"
  (cl:format cl:nil "uint16 vehicle_id~%uint16 game_status   # e.g., \"active\", \"hit\", \"penalty\", etc.~%autopilot_bridge/Geodometry state~%~%================================================================================~%MSG: autopilot_bridge/Geodometry~%# Odometry message but with GeoPoseWithCovariance instead~%~%std_msgs/Header header~%string child_frame_id~%GeoPoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: autopilot_bridge/GeoPoseWithCovariance~%# Pose using lat/lon/alt in place of x/y/z plus two altitude fields~%GeoPose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: autopilot_bridge/GeoPose~%# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RedVehicleState)))
  "Returns full string definition for message of type 'RedVehicleState"
  (cl:format cl:nil "uint16 vehicle_id~%uint16 game_status   # e.g., \"active\", \"hit\", \"penalty\", etc.~%autopilot_bridge/Geodometry state~%~%================================================================================~%MSG: autopilot_bridge/Geodometry~%# Odometry message but with GeoPoseWithCovariance instead~%~%std_msgs/Header header~%string child_frame_id~%GeoPoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: autopilot_bridge/GeoPoseWithCovariance~%# Pose using lat/lon/alt in place of x/y/z plus two altitude fields~%GeoPose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: autopilot_bridge/GeoPose~%# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RedVehicleState>))
  (cl:+ 0
     2
     2
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RedVehicleState>))
  "Converts a ROS message object to a list"
  (cl:list 'RedVehicleState
    (cl:cons ':vehicle_id (vehicle_id msg))
    (cl:cons ':game_status (game_status msg))
    (cl:cons ':state (state msg))
))
