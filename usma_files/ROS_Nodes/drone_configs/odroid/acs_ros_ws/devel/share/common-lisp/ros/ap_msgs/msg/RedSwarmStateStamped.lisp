; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude RedSwarmStateStamped.msg.html

(cl:defclass <RedSwarmStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (reds
    :reader reds
    :initarg :reds
    :type (cl:vector ap_msgs-msg:RedVehicleState)
   :initform (cl:make-array 0 :element-type 'ap_msgs-msg:RedVehicleState :initial-element (cl:make-instance 'ap_msgs-msg:RedVehicleState))))
)

(cl:defclass RedSwarmStateStamped (<RedSwarmStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RedSwarmStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RedSwarmStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<RedSwarmStateStamped> is deprecated: use ap_msgs-msg:RedSwarmStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RedSwarmStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:header-val is deprecated.  Use ap_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'reds-val :lambda-list '(m))
(cl:defmethod reds-val ((m <RedSwarmStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:reds-val is deprecated.  Use ap_msgs-msg:reds instead.")
  (reds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RedSwarmStateStamped>) ostream)
  "Serializes a message object of type '<RedSwarmStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'reds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'reds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RedSwarmStateStamped>) istream)
  "Deserializes a message object of type '<RedSwarmStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'reds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'reds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ap_msgs-msg:RedVehicleState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RedSwarmStateStamped>)))
  "Returns string type for a message object of type '<RedSwarmStateStamped>"
  "ap_msgs/RedSwarmStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RedSwarmStateStamped)))
  "Returns string type for a message object of type 'RedSwarmStateStamped"
  "ap_msgs/RedSwarmStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RedSwarmStateStamped>)))
  "Returns md5sum for a message object of type '<RedSwarmStateStamped>"
  "28ef9039eadae3d92301bc666aa6c8a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RedSwarmStateStamped)))
  "Returns md5sum for a message object of type 'RedSwarmStateStamped"
  "28ef9039eadae3d92301bc666aa6c8a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RedSwarmStateStamped>)))
  "Returns full string definition for message of type '<RedSwarmStateStamped>"
  (cl:format cl:nil "std_msgs/Header header~%ap_msgs/RedVehicleState[] reds~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ap_msgs/RedVehicleState~%uint16 vehicle_id~%uint16 game_status   # e.g., \"active\", \"hit\", \"penalty\", etc.~%autopilot_bridge/Geodometry state~%~%================================================================================~%MSG: autopilot_bridge/Geodometry~%# Odometry message but with GeoPoseWithCovariance instead~%~%std_msgs/Header header~%string child_frame_id~%GeoPoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: autopilot_bridge/GeoPoseWithCovariance~%# Pose using lat/lon/alt in place of x/y/z plus two altitude fields~%GeoPose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: autopilot_bridge/GeoPose~%# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RedSwarmStateStamped)))
  "Returns full string definition for message of type 'RedSwarmStateStamped"
  (cl:format cl:nil "std_msgs/Header header~%ap_msgs/RedVehicleState[] reds~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ap_msgs/RedVehicleState~%uint16 vehicle_id~%uint16 game_status   # e.g., \"active\", \"hit\", \"penalty\", etc.~%autopilot_bridge/Geodometry state~%~%================================================================================~%MSG: autopilot_bridge/Geodometry~%# Odometry message but with GeoPoseWithCovariance instead~%~%std_msgs/Header header~%string child_frame_id~%GeoPoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: autopilot_bridge/GeoPoseWithCovariance~%# Pose using lat/lon/alt in place of x/y/z plus two altitude fields~%GeoPose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: autopilot_bridge/GeoPose~%# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RedSwarmStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'reds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RedSwarmStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'RedSwarmStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':reds (reds msg))
))
