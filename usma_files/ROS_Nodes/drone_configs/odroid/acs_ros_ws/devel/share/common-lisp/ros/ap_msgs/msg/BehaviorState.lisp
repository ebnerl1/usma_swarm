; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude BehaviorState.msg.html

(cl:defclass <BehaviorState> (roslisp-msg-protocol:ros-message)
  ((behavior_id
    :reader behavior_id
    :initarg :behavior_id
    :type cl:fixnum
    :initform 0)
   (sequence
    :reader sequence
    :initarg :sequence
    :type cl:integer
    :initform 0)
   (is_ready
    :reader is_ready
    :initarg :is_ready
    :type cl:boolean
    :initform cl:nil)
   (is_active
    :reader is_active
    :initarg :is_active
    :type cl:boolean
    :initform cl:nil)
   (is_paused
    :reader is_paused
    :initarg :is_paused
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass BehaviorState (<BehaviorState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BehaviorState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BehaviorState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<BehaviorState> is deprecated: use ap_msgs-msg:BehaviorState instead.")))

(cl:ensure-generic-function 'behavior_id-val :lambda-list '(m))
(cl:defmethod behavior_id-val ((m <BehaviorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:behavior_id-val is deprecated.  Use ap_msgs-msg:behavior_id instead.")
  (behavior_id m))

(cl:ensure-generic-function 'sequence-val :lambda-list '(m))
(cl:defmethod sequence-val ((m <BehaviorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:sequence-val is deprecated.  Use ap_msgs-msg:sequence instead.")
  (sequence m))

(cl:ensure-generic-function 'is_ready-val :lambda-list '(m))
(cl:defmethod is_ready-val ((m <BehaviorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:is_ready-val is deprecated.  Use ap_msgs-msg:is_ready instead.")
  (is_ready m))

(cl:ensure-generic-function 'is_active-val :lambda-list '(m))
(cl:defmethod is_active-val ((m <BehaviorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:is_active-val is deprecated.  Use ap_msgs-msg:is_active instead.")
  (is_active m))

(cl:ensure-generic-function 'is_paused-val :lambda-list '(m))
(cl:defmethod is_paused-val ((m <BehaviorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:is_paused-val is deprecated.  Use ap_msgs-msg:is_paused instead.")
  (is_paused m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BehaviorState>) ostream)
  "Serializes a message object of type '<BehaviorState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'behavior_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_ready) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_active) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_paused) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BehaviorState>) istream)
  "Deserializes a message object of type '<BehaviorState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'behavior_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'is_ready) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_active) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_paused) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BehaviorState>)))
  "Returns string type for a message object of type '<BehaviorState>"
  "ap_msgs/BehaviorState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BehaviorState)))
  "Returns string type for a message object of type 'BehaviorState"
  "ap_msgs/BehaviorState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BehaviorState>)))
  "Returns md5sum for a message object of type '<BehaviorState>"
  "1c5487d0efa12e59fc9bb8960f39f1a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BehaviorState)))
  "Returns md5sum for a message object of type 'BehaviorState"
  "1c5487d0efa12e59fc9bb8960f39f1a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BehaviorState>)))
  "Returns full string definition for message of type '<BehaviorState>"
  (cl:format cl:nil "uint8  behavior_id~%uint32 sequence~%bool   is_ready~%bool   is_active~%bool   is_paused~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BehaviorState)))
  "Returns full string definition for message of type 'BehaviorState"
  (cl:format cl:nil "uint8  behavior_id~%uint32 sequence~%bool   is_ready~%bool   is_active~%bool   is_paused~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BehaviorState>))
  (cl:+ 0
     1
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BehaviorState>))
  "Converts a ROS message object to a list"
  (cl:list 'BehaviorState
    (cl:cons ':behavior_id (behavior_id msg))
    (cl:cons ':sequence (sequence msg))
    (cl:cons ':is_ready (is_ready msg))
    (cl:cons ':is_active (is_active msg))
    (cl:cons ':is_paused (is_paused msg))
))
