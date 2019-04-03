; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude SwarmVehicleState.msg.html

(cl:defclass <SwarmVehicleState> (roslisp-msg-protocol:ros-message)
  ((vehicle_id
    :reader vehicle_id
    :initarg :vehicle_id
    :type cl:fixnum
    :initform 0)
   (subswarm_id
    :reader subswarm_id
    :initarg :subswarm_id
    :type cl:fixnum
    :initform 0)
   (received_at
    :reader received_at
    :initarg :received_at
    :type cl:real
    :initform 0)
   (swarm_state
    :reader swarm_state
    :initarg :swarm_state
    :type cl:fixnum
    :initform 0)
   (swarm_behavior
    :reader swarm_behavior
    :initarg :swarm_behavior
    :type cl:fixnum
    :initform 0)
   (vehicle_type
    :reader vehicle_type
    :initarg :vehicle_type
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type autopilot_bridge-msg:Geodometry
    :initform (cl:make-instance 'autopilot_bridge-msg:Geodometry)))
)

(cl:defclass SwarmVehicleState (<SwarmVehicleState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwarmVehicleState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwarmVehicleState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<SwarmVehicleState> is deprecated: use ap_msgs-msg:SwarmVehicleState instead.")))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <SwarmVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:vehicle_id-val is deprecated.  Use ap_msgs-msg:vehicle_id instead.")
  (vehicle_id m))

(cl:ensure-generic-function 'subswarm_id-val :lambda-list '(m))
(cl:defmethod subswarm_id-val ((m <SwarmVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:subswarm_id-val is deprecated.  Use ap_msgs-msg:subswarm_id instead.")
  (subswarm_id m))

(cl:ensure-generic-function 'received_at-val :lambda-list '(m))
(cl:defmethod received_at-val ((m <SwarmVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:received_at-val is deprecated.  Use ap_msgs-msg:received_at instead.")
  (received_at m))

(cl:ensure-generic-function 'swarm_state-val :lambda-list '(m))
(cl:defmethod swarm_state-val ((m <SwarmVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:swarm_state-val is deprecated.  Use ap_msgs-msg:swarm_state instead.")
  (swarm_state m))

(cl:ensure-generic-function 'swarm_behavior-val :lambda-list '(m))
(cl:defmethod swarm_behavior-val ((m <SwarmVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:swarm_behavior-val is deprecated.  Use ap_msgs-msg:swarm_behavior instead.")
  (swarm_behavior m))

(cl:ensure-generic-function 'vehicle_type-val :lambda-list '(m))
(cl:defmethod vehicle_type-val ((m <SwarmVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:vehicle_type-val is deprecated.  Use ap_msgs-msg:vehicle_type instead.")
  (vehicle_type m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SwarmVehicleState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:state-val is deprecated.  Use ap_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwarmVehicleState>) ostream)
  "Serializes a message object of type '<SwarmVehicleState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subswarm_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subswarm_id)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'received_at)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'received_at) (cl:floor (cl:slot-value msg 'received_at)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_behavior)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwarmVehicleState>) istream)
  "Deserializes a message object of type '<SwarmVehicleState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subswarm_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subswarm_id)) (cl:read-byte istream))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'received_at) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_behavior)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwarmVehicleState>)))
  "Returns string type for a message object of type '<SwarmVehicleState>"
  "ap_msgs/SwarmVehicleState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwarmVehicleState)))
  "Returns string type for a message object of type 'SwarmVehicleState"
  "ap_msgs/SwarmVehicleState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwarmVehicleState>)))
  "Returns md5sum for a message object of type '<SwarmVehicleState>"
  "6aa4127caa574d154d4fadf1234e6504")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwarmVehicleState)))
  "Returns md5sum for a message object of type 'SwarmVehicleState"
  "6aa4127caa574d154d4fadf1234e6504")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwarmVehicleState>)))
  "Returns full string definition for message of type '<SwarmVehicleState>"
  (cl:format cl:nil "uint16 vehicle_id~%uint16 subswarm_id~%time received_at~%uint8 swarm_state~%uint8 swarm_behavior~%uint8 vehicle_type~%autopilot_bridge/Geodometry state~%~%================================================================================~%MSG: autopilot_bridge/Geodometry~%# Odometry message but with GeoPoseWithCovariance instead~%~%std_msgs/Header header~%string child_frame_id~%GeoPoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: autopilot_bridge/GeoPoseWithCovariance~%# Pose using lat/lon/alt in place of x/y/z plus two altitude fields~%GeoPose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: autopilot_bridge/GeoPose~%# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwarmVehicleState)))
  "Returns full string definition for message of type 'SwarmVehicleState"
  (cl:format cl:nil "uint16 vehicle_id~%uint16 subswarm_id~%time received_at~%uint8 swarm_state~%uint8 swarm_behavior~%uint8 vehicle_type~%autopilot_bridge/Geodometry state~%~%================================================================================~%MSG: autopilot_bridge/Geodometry~%# Odometry message but with GeoPoseWithCovariance instead~%~%std_msgs/Header header~%string child_frame_id~%GeoPoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: autopilot_bridge/GeoPoseWithCovariance~%# Pose using lat/lon/alt in place of x/y/z plus two altitude fields~%GeoPose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: autopilot_bridge/GeoPose~%# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertainty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwarmVehicleState>))
  (cl:+ 0
     2
     2
     8
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwarmVehicleState>))
  "Converts a ROS message object to a list"
  (cl:list 'SwarmVehicleState
    (cl:cons ':vehicle_id (vehicle_id msg))
    (cl:cons ':subswarm_id (subswarm_id msg))
    (cl:cons ':received_at (received_at msg))
    (cl:cons ':swarm_state (swarm_state msg))
    (cl:cons ':swarm_behavior (swarm_behavior msg))
    (cl:cons ':vehicle_type (vehicle_type msg))
    (cl:cons ':state (state msg))
))
