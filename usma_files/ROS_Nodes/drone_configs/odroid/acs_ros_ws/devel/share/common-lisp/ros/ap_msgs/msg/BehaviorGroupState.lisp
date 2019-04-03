; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude BehaviorGroupState.msg.html

(cl:defclass <BehaviorGroupState> (roslisp-msg-protocol:ros-message)
  ((active_behavior
    :reader active_behavior
    :initarg :active_behavior
    :type cl:fixnum
    :initform 0)
   (behaviors
    :reader behaviors
    :initarg :behaviors
    :type (cl:vector ap_msgs-msg:BehaviorState)
   :initform (cl:make-array 0 :element-type 'ap_msgs-msg:BehaviorState :initial-element (cl:make-instance 'ap_msgs-msg:BehaviorState))))
)

(cl:defclass BehaviorGroupState (<BehaviorGroupState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BehaviorGroupState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BehaviorGroupState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<BehaviorGroupState> is deprecated: use ap_msgs-msg:BehaviorGroupState instead.")))

(cl:ensure-generic-function 'active_behavior-val :lambda-list '(m))
(cl:defmethod active_behavior-val ((m <BehaviorGroupState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:active_behavior-val is deprecated.  Use ap_msgs-msg:active_behavior instead.")
  (active_behavior m))

(cl:ensure-generic-function 'behaviors-val :lambda-list '(m))
(cl:defmethod behaviors-val ((m <BehaviorGroupState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:behaviors-val is deprecated.  Use ap_msgs-msg:behaviors instead.")
  (behaviors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BehaviorGroupState>) ostream)
  "Serializes a message object of type '<BehaviorGroupState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_behavior)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'behaviors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'behaviors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BehaviorGroupState>) istream)
  "Deserializes a message object of type '<BehaviorGroupState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_behavior)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'behaviors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'behaviors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ap_msgs-msg:BehaviorState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BehaviorGroupState>)))
  "Returns string type for a message object of type '<BehaviorGroupState>"
  "ap_msgs/BehaviorGroupState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BehaviorGroupState)))
  "Returns string type for a message object of type 'BehaviorGroupState"
  "ap_msgs/BehaviorGroupState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BehaviorGroupState>)))
  "Returns md5sum for a message object of type '<BehaviorGroupState>"
  "61c10d40a72ba1aa819a6e0972f5cd9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BehaviorGroupState)))
  "Returns md5sum for a message object of type 'BehaviorGroupState"
  "61c10d40a72ba1aa819a6e0972f5cd9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BehaviorGroupState>)))
  "Returns full string definition for message of type '<BehaviorGroupState>"
  (cl:format cl:nil "uint8 active_behavior~%ap_msgs/BehaviorState[] behaviors~%~%================================================================================~%MSG: ap_msgs/BehaviorState~%uint8  behavior_id~%uint32 sequence~%bool   is_ready~%bool   is_active~%bool   is_paused~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BehaviorGroupState)))
  "Returns full string definition for message of type 'BehaviorGroupState"
  (cl:format cl:nil "uint8 active_behavior~%ap_msgs/BehaviorState[] behaviors~%~%================================================================================~%MSG: ap_msgs/BehaviorState~%uint8  behavior_id~%uint32 sequence~%bool   is_ready~%bool   is_active~%bool   is_paused~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BehaviorGroupState>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'behaviors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BehaviorGroupState>))
  "Converts a ROS message object to a list"
  (cl:list 'BehaviorGroupState
    (cl:cons ':active_behavior (active_behavior msg))
    (cl:cons ':behaviors (behaviors msg))
))
