; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude MsgStatList.msg.html

(cl:defclass <MsgStatList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (stat
    :reader stat
    :initarg :stat
    :type (cl:vector ap_msgs-msg:MsgStat)
   :initform (cl:make-array 0 :element-type 'ap_msgs-msg:MsgStat :initial-element (cl:make-instance 'ap_msgs-msg:MsgStat))))
)

(cl:defclass MsgStatList (<MsgStatList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MsgStatList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MsgStatList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<MsgStatList> is deprecated: use ap_msgs-msg:MsgStatList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MsgStatList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:header-val is deprecated.  Use ap_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'stat-val :lambda-list '(m))
(cl:defmethod stat-val ((m <MsgStatList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:stat-val is deprecated.  Use ap_msgs-msg:stat instead.")
  (stat m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MsgStatList>) ostream)
  "Serializes a message object of type '<MsgStatList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'stat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'stat))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MsgStatList>) istream)
  "Deserializes a message object of type '<MsgStatList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'stat) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'stat)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ap_msgs-msg:MsgStat))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MsgStatList>)))
  "Returns string type for a message object of type '<MsgStatList>"
  "ap_msgs/MsgStatList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MsgStatList)))
  "Returns string type for a message object of type 'MsgStatList"
  "ap_msgs/MsgStatList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MsgStatList>)))
  "Returns md5sum for a message object of type '<MsgStatList>"
  "cf6d730c2e3f02162fd3839b8e723db1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MsgStatList)))
  "Returns md5sum for a message object of type 'MsgStatList"
  "cf6d730c2e3f02162fd3839b8e723db1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MsgStatList>)))
  "Returns full string definition for message of type '<MsgStatList>"
  (cl:format cl:nil "std_msgs/Header header~%ap_msgs/MsgStat[] stat~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ap_msgs/MsgStat~%uint16 id~%uint32 count~%float64 latency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MsgStatList)))
  "Returns full string definition for message of type 'MsgStatList"
  (cl:format cl:nil "std_msgs/Header header~%ap_msgs/MsgStat[] stat~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ap_msgs/MsgStat~%uint16 id~%uint32 count~%float64 latency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MsgStatList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'stat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MsgStatList>))
  "Converts a ROS message object to a list"
  (cl:list 'MsgStatList
    (cl:cons ':header (header msg))
    (cl:cons ':stat (stat msg))
))
