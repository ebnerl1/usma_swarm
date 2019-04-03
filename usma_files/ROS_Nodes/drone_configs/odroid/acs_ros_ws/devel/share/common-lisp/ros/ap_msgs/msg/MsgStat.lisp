; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude MsgStat.msg.html

(cl:defclass <MsgStat> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0)
   (latency
    :reader latency
    :initarg :latency
    :type cl:float
    :initform 0.0))
)

(cl:defclass MsgStat (<MsgStat>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MsgStat>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MsgStat)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<MsgStat> is deprecated: use ap_msgs-msg:MsgStat instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <MsgStat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:id-val is deprecated.  Use ap_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <MsgStat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:count-val is deprecated.  Use ap_msgs-msg:count instead.")
  (count m))

(cl:ensure-generic-function 'latency-val :lambda-list '(m))
(cl:defmethod latency-val ((m <MsgStat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:latency-val is deprecated.  Use ap_msgs-msg:latency instead.")
  (latency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MsgStat>) ostream)
  "Serializes a message object of type '<MsgStat>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'count)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MsgStat>) istream)
  "Deserializes a message object of type '<MsgStat>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latency) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MsgStat>)))
  "Returns string type for a message object of type '<MsgStat>"
  "ap_msgs/MsgStat")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MsgStat)))
  "Returns string type for a message object of type 'MsgStat"
  "ap_msgs/MsgStat")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MsgStat>)))
  "Returns md5sum for a message object of type '<MsgStat>"
  "7b03b0e59c2cf7eb753d1ca3563758d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MsgStat)))
  "Returns md5sum for a message object of type 'MsgStat"
  "7b03b0e59c2cf7eb753d1ca3563758d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MsgStat>)))
  "Returns full string definition for message of type '<MsgStat>"
  (cl:format cl:nil "uint16 id~%uint32 count~%float64 latency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MsgStat)))
  "Returns full string definition for message of type 'MsgStat"
  (cl:format cl:nil "uint16 id~%uint32 count~%float64 latency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MsgStat>))
  (cl:+ 0
     2
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MsgStat>))
  "Converts a ROS message object to a list"
  (cl:list 'MsgStat
    (cl:cons ':id (id msg))
    (cl:cons ':count (count msg))
    (cl:cons ':latency (latency msg))
))
