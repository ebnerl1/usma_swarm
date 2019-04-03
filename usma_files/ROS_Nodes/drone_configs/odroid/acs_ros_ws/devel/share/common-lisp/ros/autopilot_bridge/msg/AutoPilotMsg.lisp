; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-msg)


;//! \htmlinclude AutoPilotMsg.msg.html

(cl:defclass <AutoPilotMsg> (roslisp-msg-protocol:ros-message)
  ((seq
    :reader seq
    :initarg :seq
    :type cl:integer
    :initform 0)
   (text
    :reader text
    :initarg :text
    :type cl:string
    :initform ""))
)

(cl:defclass AutoPilotMsg (<AutoPilotMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AutoPilotMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AutoPilotMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-msg:<AutoPilotMsg> is deprecated: use autopilot_bridge-msg:AutoPilotMsg instead.")))

(cl:ensure-generic-function 'seq-val :lambda-list '(m))
(cl:defmethod seq-val ((m <AutoPilotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:seq-val is deprecated.  Use autopilot_bridge-msg:seq instead.")
  (seq m))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <AutoPilotMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:text-val is deprecated.  Use autopilot_bridge-msg:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AutoPilotMsg>) ostream)
  "Serializes a message object of type '<AutoPilotMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AutoPilotMsg>) istream)
  "Deserializes a message object of type '<AutoPilotMsg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'seq)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AutoPilotMsg>)))
  "Returns string type for a message object of type '<AutoPilotMsg>"
  "autopilot_bridge/AutoPilotMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AutoPilotMsg)))
  "Returns string type for a message object of type 'AutoPilotMsg"
  "autopilot_bridge/AutoPilotMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AutoPilotMsg>)))
  "Returns md5sum for a message object of type '<AutoPilotMsg>"
  "4a6e7dd37ede14708a8dd0871344bc2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AutoPilotMsg)))
  "Returns md5sum for a message object of type 'AutoPilotMsg"
  "4a6e7dd37ede14708a8dd0871344bc2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AutoPilotMsg>)))
  "Returns full string definition for message of type '<AutoPilotMsg>"
  (cl:format cl:nil "uint32  seq~%string  text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AutoPilotMsg)))
  "Returns full string definition for message of type 'AutoPilotMsg"
  (cl:format cl:nil "uint32  seq~%string  text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AutoPilotMsg>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AutoPilotMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'AutoPilotMsg
    (cl:cons ':seq (seq msg))
    (cl:cons ':text (text msg))
))
