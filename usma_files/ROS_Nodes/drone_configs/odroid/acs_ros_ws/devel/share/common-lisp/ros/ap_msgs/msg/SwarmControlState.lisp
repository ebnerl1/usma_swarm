; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude SwarmControlState.msg.html

(cl:defclass <SwarmControlState> (roslisp-msg-protocol:ros-message)
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
   (autopilot_mode
    :reader autopilot_mode
    :initarg :autopilot_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SwarmControlState (<SwarmControlState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwarmControlState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwarmControlState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<SwarmControlState> is deprecated: use ap_msgs-msg:SwarmControlState instead.")))

(cl:ensure-generic-function 'vehicle_id-val :lambda-list '(m))
(cl:defmethod vehicle_id-val ((m <SwarmControlState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:vehicle_id-val is deprecated.  Use ap_msgs-msg:vehicle_id instead.")
  (vehicle_id m))

(cl:ensure-generic-function 'subswarm_id-val :lambda-list '(m))
(cl:defmethod subswarm_id-val ((m <SwarmControlState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:subswarm_id-val is deprecated.  Use ap_msgs-msg:subswarm_id instead.")
  (subswarm_id m))

(cl:ensure-generic-function 'swarm_state-val :lambda-list '(m))
(cl:defmethod swarm_state-val ((m <SwarmControlState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:swarm_state-val is deprecated.  Use ap_msgs-msg:swarm_state instead.")
  (swarm_state m))

(cl:ensure-generic-function 'swarm_behavior-val :lambda-list '(m))
(cl:defmethod swarm_behavior-val ((m <SwarmControlState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:swarm_behavior-val is deprecated.  Use ap_msgs-msg:swarm_behavior instead.")
  (swarm_behavior m))

(cl:ensure-generic-function 'autopilot_mode-val :lambda-list '(m))
(cl:defmethod autopilot_mode-val ((m <SwarmControlState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:autopilot_mode-val is deprecated.  Use ap_msgs-msg:autopilot_mode instead.")
  (autopilot_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwarmControlState>) ostream)
  "Serializes a message object of type '<SwarmControlState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subswarm_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_behavior)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'autopilot_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwarmControlState>) istream)
  "Deserializes a message object of type '<SwarmControlState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subswarm_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'swarm_behavior)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'autopilot_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwarmControlState>)))
  "Returns string type for a message object of type '<SwarmControlState>"
  "ap_msgs/SwarmControlState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwarmControlState)))
  "Returns string type for a message object of type 'SwarmControlState"
  "ap_msgs/SwarmControlState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwarmControlState>)))
  "Returns md5sum for a message object of type '<SwarmControlState>"
  "b2d72190b6e1a63abcc19587ee666fbf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwarmControlState)))
  "Returns md5sum for a message object of type 'SwarmControlState"
  "b2d72190b6e1a63abcc19587ee666fbf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwarmControlState>)))
  "Returns full string definition for message of type '<SwarmControlState>"
  (cl:format cl:nil "uint8 vehicle_id~%uint8 subswarm_id~%uint8 swarm_state~%uint8 swarm_behavior~%uint8 autopilot_mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwarmControlState)))
  "Returns full string definition for message of type 'SwarmControlState"
  (cl:format cl:nil "uint8 vehicle_id~%uint8 subswarm_id~%uint8 swarm_state~%uint8 swarm_behavior~%uint8 autopilot_mode~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwarmControlState>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwarmControlState>))
  "Converts a ROS message object to a list"
  (cl:list 'SwarmControlState
    (cl:cons ':vehicle_id (vehicle_id msg))
    (cl:cons ':subswarm_id (subswarm_id msg))
    (cl:cons ':swarm_state (swarm_state msg))
    (cl:cons ':swarm_behavior (swarm_behavior msg))
    (cl:cons ':autopilot_mode (autopilot_mode msg))
))
