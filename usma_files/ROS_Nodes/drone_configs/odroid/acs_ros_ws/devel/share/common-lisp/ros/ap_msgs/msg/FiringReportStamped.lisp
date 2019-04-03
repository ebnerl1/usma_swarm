; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude FiringReportStamped.msg.html

(cl:defclass <FiringReportStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (report
    :reader report
    :initarg :report
    :type ap_msgs-msg:FiringReport
    :initform (cl:make-instance 'ap_msgs-msg:FiringReport)))
)

(cl:defclass FiringReportStamped (<FiringReportStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FiringReportStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FiringReportStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<FiringReportStamped> is deprecated: use ap_msgs-msg:FiringReportStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FiringReportStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:header-val is deprecated.  Use ap_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'report-val :lambda-list '(m))
(cl:defmethod report-val ((m <FiringReportStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:report-val is deprecated.  Use ap_msgs-msg:report instead.")
  (report m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FiringReportStamped>) ostream)
  "Serializes a message object of type '<FiringReportStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'report) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FiringReportStamped>) istream)
  "Deserializes a message object of type '<FiringReportStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'report) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FiringReportStamped>)))
  "Returns string type for a message object of type '<FiringReportStamped>"
  "ap_msgs/FiringReportStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FiringReportStamped)))
  "Returns string type for a message object of type 'FiringReportStamped"
  "ap_msgs/FiringReportStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FiringReportStamped>)))
  "Returns md5sum for a message object of type '<FiringReportStamped>"
  "3615f1e990118110730c88488f2ec016")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FiringReportStamped)))
  "Returns md5sum for a message object of type 'FiringReportStamped"
  "3615f1e990118110730c88488f2ec016")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FiringReportStamped>)))
  "Returns full string definition for message of type '<FiringReportStamped>"
  (cl:format cl:nil "std_msgs/Header header~%ap_msgs/FiringReport report~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ap_msgs/FiringReport~%float32 seq~%float32 lat~%float32 lon~%float32 alt~%float32 yaw~%uint8 target_id~%float32 target_lat~%float32 target_lon~%float32 target_alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FiringReportStamped)))
  "Returns full string definition for message of type 'FiringReportStamped"
  (cl:format cl:nil "std_msgs/Header header~%ap_msgs/FiringReport report~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ap_msgs/FiringReport~%float32 seq~%float32 lat~%float32 lon~%float32 alt~%float32 yaw~%uint8 target_id~%float32 target_lat~%float32 target_lon~%float32 target_alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FiringReportStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'report))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FiringReportStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'FiringReportStamped
    (cl:cons ':header (header msg))
    (cl:cons ':report (report msg))
))
