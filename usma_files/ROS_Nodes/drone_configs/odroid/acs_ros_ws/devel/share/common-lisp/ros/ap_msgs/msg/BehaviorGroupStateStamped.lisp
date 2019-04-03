; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude BehaviorGroupStateStamped.msg.html

(cl:defclass <BehaviorGroupStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type ap_msgs-msg:BehaviorGroupState
    :initform (cl:make-instance 'ap_msgs-msg:BehaviorGroupState)))
)

(cl:defclass BehaviorGroupStateStamped (<BehaviorGroupStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BehaviorGroupStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BehaviorGroupStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<BehaviorGroupStateStamped> is deprecated: use ap_msgs-msg:BehaviorGroupStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BehaviorGroupStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:header-val is deprecated.  Use ap_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <BehaviorGroupStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:state-val is deprecated.  Use ap_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BehaviorGroupStateStamped>) ostream)
  "Serializes a message object of type '<BehaviorGroupStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BehaviorGroupStateStamped>) istream)
  "Deserializes a message object of type '<BehaviorGroupStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BehaviorGroupStateStamped>)))
  "Returns string type for a message object of type '<BehaviorGroupStateStamped>"
  "ap_msgs/BehaviorGroupStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BehaviorGroupStateStamped)))
  "Returns string type for a message object of type 'BehaviorGroupStateStamped"
  "ap_msgs/BehaviorGroupStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BehaviorGroupStateStamped>)))
  "Returns md5sum for a message object of type '<BehaviorGroupStateStamped>"
  "3170e7586511ce336a1e2d4f90a8ba5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BehaviorGroupStateStamped)))
  "Returns md5sum for a message object of type 'BehaviorGroupStateStamped"
  "3170e7586511ce336a1e2d4f90a8ba5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BehaviorGroupStateStamped>)))
  "Returns full string definition for message of type '<BehaviorGroupStateStamped>"
  (cl:format cl:nil "std_msgs/Header header~%ap_msgs/BehaviorGroupState state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ap_msgs/BehaviorGroupState~%uint8 active_behavior~%ap_msgs/BehaviorState[] behaviors~%~%================================================================================~%MSG: ap_msgs/BehaviorState~%uint8  behavior_id~%uint32 sequence~%bool   is_ready~%bool   is_active~%bool   is_paused~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BehaviorGroupStateStamped)))
  "Returns full string definition for message of type 'BehaviorGroupStateStamped"
  (cl:format cl:nil "std_msgs/Header header~%ap_msgs/BehaviorGroupState state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ap_msgs/BehaviorGroupState~%uint8 active_behavior~%ap_msgs/BehaviorState[] behaviors~%~%================================================================================~%MSG: ap_msgs/BehaviorState~%uint8  behavior_id~%uint32 sequence~%bool   is_ready~%bool   is_active~%bool   is_paused~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BehaviorGroupStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BehaviorGroupStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'BehaviorGroupStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
))
