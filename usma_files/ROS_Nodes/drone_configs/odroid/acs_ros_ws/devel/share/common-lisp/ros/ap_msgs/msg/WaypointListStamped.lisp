; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude WaypointListStamped.msg.html

(cl:defclass <WaypointListStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector autopilot_bridge-msg:LLA)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:LLA :initial-element (cl:make-instance 'autopilot_bridge-msg:LLA))))
)

(cl:defclass WaypointListStamped (<WaypointListStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointListStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointListStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<WaypointListStamped> is deprecated: use ap_msgs-msg:WaypointListStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WaypointListStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:header-val is deprecated.  Use ap_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <WaypointListStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:waypoints-val is deprecated.  Use ap_msgs-msg:waypoints instead.")
  (waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointListStamped>) ostream)
  "Serializes a message object of type '<WaypointListStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointListStamped>) istream)
  "Deserializes a message object of type '<WaypointListStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'autopilot_bridge-msg:LLA))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointListStamped>)))
  "Returns string type for a message object of type '<WaypointListStamped>"
  "ap_msgs/WaypointListStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointListStamped)))
  "Returns string type for a message object of type 'WaypointListStamped"
  "ap_msgs/WaypointListStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointListStamped>)))
  "Returns md5sum for a message object of type '<WaypointListStamped>"
  "f67e9f276998486487ac3641f5ef5965")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointListStamped)))
  "Returns md5sum for a message object of type 'WaypointListStamped"
  "f67e9f276998486487ac3641f5ef5965")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointListStamped>)))
  "Returns full string definition for message of type '<WaypointListStamped>"
  (cl:format cl:nil "std_msgs/Header header~%autopilot_bridge/LLA[] waypoints~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: autopilot_bridge/LLA~%float64 lat~%float64 lon~%float64 alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointListStamped)))
  "Returns full string definition for message of type 'WaypointListStamped"
  (cl:format cl:nil "std_msgs/Header header~%autopilot_bridge/LLA[] waypoints~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: autopilot_bridge/LLA~%float64 lat~%float64 lon~%float64 alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointListStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointListStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointListStamped
    (cl:cons ':header (header msg))
    (cl:cons ':waypoints (waypoints msg))
))
