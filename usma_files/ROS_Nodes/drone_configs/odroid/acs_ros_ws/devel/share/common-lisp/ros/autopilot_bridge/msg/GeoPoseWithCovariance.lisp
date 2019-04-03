; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-msg)


;//! \htmlinclude GeoPoseWithCovariance.msg.html

(cl:defclass <GeoPoseWithCovariance> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type autopilot_bridge-msg:GeoPose
    :initform (cl:make-instance 'autopilot_bridge-msg:GeoPose))
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 36 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GeoPoseWithCovariance (<GeoPoseWithCovariance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GeoPoseWithCovariance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GeoPoseWithCovariance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-msg:<GeoPoseWithCovariance> is deprecated: use autopilot_bridge-msg:GeoPoseWithCovariance instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GeoPoseWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:pose-val is deprecated.  Use autopilot_bridge-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <GeoPoseWithCovariance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:covariance-val is deprecated.  Use autopilot_bridge-msg:covariance instead.")
  (covariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GeoPoseWithCovariance>) ostream)
  "Serializes a message object of type '<GeoPoseWithCovariance>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GeoPoseWithCovariance>) istream)
  "Deserializes a message object of type '<GeoPoseWithCovariance>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 36))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 36)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GeoPoseWithCovariance>)))
  "Returns string type for a message object of type '<GeoPoseWithCovariance>"
  "autopilot_bridge/GeoPoseWithCovariance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GeoPoseWithCovariance)))
  "Returns string type for a message object of type 'GeoPoseWithCovariance"
  "autopilot_bridge/GeoPoseWithCovariance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GeoPoseWithCovariance>)))
  "Returns md5sum for a message object of type '<GeoPoseWithCovariance>"
  "49607a12d5ec103bbc7dc81cc88d8763")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GeoPoseWithCovariance)))
  "Returns md5sum for a message object of type 'GeoPoseWithCovariance"
  "49607a12d5ec103bbc7dc81cc88d8763")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GeoPoseWithCovariance>)))
  "Returns full string definition for message of type '<GeoPoseWithCovariance>"
  (cl:format cl:nil "# Pose using lat/lon/alt in place of x/y/z plus two altitude fields~%GeoPose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: autopilot_bridge/GeoPose~%# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GeoPoseWithCovariance)))
  "Returns full string definition for message of type 'GeoPoseWithCovariance"
  (cl:format cl:nil "# Pose using lat/lon/alt in place of x/y/z plus two altitude fields~%GeoPose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: autopilot_bridge/GeoPose~%# 3-D point using lat/lon/alt, with two altitude options~%GeoPoint position~%~%# Quaternion orientation~%geometry_msgs/Quaternion orientation~%~%================================================================================~%MSG: autopilot_bridge/GeoPoint~%# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GeoPoseWithCovariance>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GeoPoseWithCovariance>))
  "Converts a ROS message object to a list"
  (cl:list 'GeoPoseWithCovariance
    (cl:cons ':pose (pose msg))
    (cl:cons ':covariance (covariance msg))
))
