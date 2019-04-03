; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude Version-request.msg.html

(cl:defclass <Version-request> (roslisp-msg-protocol:ros-message)
  ((timeout
    :reader timeout
    :initarg :timeout
    :type cl:float
    :initform 0.0))
)

(cl:defclass Version-request (<Version-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Version-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Version-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<Version-request> is deprecated: use autopilot_bridge-srv:Version-request instead.")))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <Version-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:timeout-val is deprecated.  Use autopilot_bridge-srv:timeout instead.")
  (timeout m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Version-request>) ostream)
  "Serializes a message object of type '<Version-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'timeout))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Version-request>) istream)
  "Deserializes a message object of type '<Version-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeout) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Version-request>)))
  "Returns string type for a service object of type '<Version-request>"
  "autopilot_bridge/VersionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Version-request)))
  "Returns string type for a service object of type 'Version-request"
  "autopilot_bridge/VersionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Version-request>)))
  "Returns md5sum for a message object of type '<Version-request>"
  "d45d1d0a68288be9b762029a29740e01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Version-request)))
  "Returns md5sum for a message object of type 'Version-request"
  "d45d1d0a68288be9b762029a29740e01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Version-request>)))
  "Returns full string definition for message of type '<Version-request>"
  (cl:format cl:nil "float32 timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Version-request)))
  "Returns full string definition for message of type 'Version-request"
  (cl:format cl:nil "float32 timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Version-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Version-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Version-request
    (cl:cons ':timeout (timeout msg))
))
;//! \htmlinclude Version-response.msg.html

(cl:defclass <Version-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (cap_mission_float
    :reader cap_mission_float
    :initarg :cap_mission_float
    :type cl:boolean
    :initform cl:nil)
   (cap_param_float
    :reader cap_param_float
    :initarg :cap_param_float
    :type cl:boolean
    :initform cl:nil)
   (cap_mission_int
    :reader cap_mission_int
    :initarg :cap_mission_int
    :type cl:boolean
    :initform cl:nil)
   (cap_command_int
    :reader cap_command_int
    :initarg :cap_command_int
    :type cl:boolean
    :initform cl:nil)
   (cap_param_union
    :reader cap_param_union
    :initarg :cap_param_union
    :type cl:boolean
    :initform cl:nil)
   (cap_ftp
    :reader cap_ftp
    :initarg :cap_ftp
    :type cl:boolean
    :initform cl:nil)
   (cap_set_attitude
    :reader cap_set_attitude
    :initarg :cap_set_attitude
    :type cl:boolean
    :initform cl:nil)
   (cap_set_position_local
    :reader cap_set_position_local
    :initarg :cap_set_position_local
    :type cl:boolean
    :initform cl:nil)
   (cap_set_position_global
    :reader cap_set_position_global
    :initarg :cap_set_position_global
    :type cl:boolean
    :initform cl:nil)
   (cap_terrain
    :reader cap_terrain
    :initarg :cap_terrain
    :type cl:boolean
    :initform cl:nil)
   (cap_set_actuator
    :reader cap_set_actuator
    :initarg :cap_set_actuator
    :type cl:boolean
    :initform cl:nil)
   (flight_sw
    :reader flight_sw
    :initarg :flight_sw
    :type cl:integer
    :initform 0)
   (middleware
    :reader middleware
    :initarg :middleware
    :type cl:integer
    :initform 0)
   (os_sw
    :reader os_sw
    :initarg :os_sw
    :type cl:integer
    :initform 0)
   (board
    :reader board
    :initarg :board
    :type cl:integer
    :initform 0)
   (flight_custom
    :reader flight_custom
    :initarg :flight_custom
    :type cl:string
    :initform "")
   (middleware_custom
    :reader middleware_custom
    :initarg :middleware_custom
    :type cl:string
    :initform "")
   (os_custom
    :reader os_custom
    :initarg :os_custom
    :type cl:string
    :initform "")
   (vendor_id
    :reader vendor_id
    :initarg :vendor_id
    :type cl:fixnum
    :initform 0)
   (product_id
    :reader product_id
    :initarg :product_id
    :type cl:fixnum
    :initform 0)
   (uid
    :reader uid
    :initarg :uid
    :type cl:integer
    :initform 0))
)

(cl:defclass Version-response (<Version-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Version-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Version-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<Version-response> is deprecated: use autopilot_bridge-srv:Version-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'cap_mission_float-val :lambda-list '(m))
(cl:defmethod cap_mission_float-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_mission_float-val is deprecated.  Use autopilot_bridge-srv:cap_mission_float instead.")
  (cap_mission_float m))

(cl:ensure-generic-function 'cap_param_float-val :lambda-list '(m))
(cl:defmethod cap_param_float-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_param_float-val is deprecated.  Use autopilot_bridge-srv:cap_param_float instead.")
  (cap_param_float m))

(cl:ensure-generic-function 'cap_mission_int-val :lambda-list '(m))
(cl:defmethod cap_mission_int-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_mission_int-val is deprecated.  Use autopilot_bridge-srv:cap_mission_int instead.")
  (cap_mission_int m))

(cl:ensure-generic-function 'cap_command_int-val :lambda-list '(m))
(cl:defmethod cap_command_int-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_command_int-val is deprecated.  Use autopilot_bridge-srv:cap_command_int instead.")
  (cap_command_int m))

(cl:ensure-generic-function 'cap_param_union-val :lambda-list '(m))
(cl:defmethod cap_param_union-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_param_union-val is deprecated.  Use autopilot_bridge-srv:cap_param_union instead.")
  (cap_param_union m))

(cl:ensure-generic-function 'cap_ftp-val :lambda-list '(m))
(cl:defmethod cap_ftp-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_ftp-val is deprecated.  Use autopilot_bridge-srv:cap_ftp instead.")
  (cap_ftp m))

(cl:ensure-generic-function 'cap_set_attitude-val :lambda-list '(m))
(cl:defmethod cap_set_attitude-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_set_attitude-val is deprecated.  Use autopilot_bridge-srv:cap_set_attitude instead.")
  (cap_set_attitude m))

(cl:ensure-generic-function 'cap_set_position_local-val :lambda-list '(m))
(cl:defmethod cap_set_position_local-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_set_position_local-val is deprecated.  Use autopilot_bridge-srv:cap_set_position_local instead.")
  (cap_set_position_local m))

(cl:ensure-generic-function 'cap_set_position_global-val :lambda-list '(m))
(cl:defmethod cap_set_position_global-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_set_position_global-val is deprecated.  Use autopilot_bridge-srv:cap_set_position_global instead.")
  (cap_set_position_global m))

(cl:ensure-generic-function 'cap_terrain-val :lambda-list '(m))
(cl:defmethod cap_terrain-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_terrain-val is deprecated.  Use autopilot_bridge-srv:cap_terrain instead.")
  (cap_terrain m))

(cl:ensure-generic-function 'cap_set_actuator-val :lambda-list '(m))
(cl:defmethod cap_set_actuator-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:cap_set_actuator-val is deprecated.  Use autopilot_bridge-srv:cap_set_actuator instead.")
  (cap_set_actuator m))

(cl:ensure-generic-function 'flight_sw-val :lambda-list '(m))
(cl:defmethod flight_sw-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:flight_sw-val is deprecated.  Use autopilot_bridge-srv:flight_sw instead.")
  (flight_sw m))

(cl:ensure-generic-function 'middleware-val :lambda-list '(m))
(cl:defmethod middleware-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:middleware-val is deprecated.  Use autopilot_bridge-srv:middleware instead.")
  (middleware m))

(cl:ensure-generic-function 'os_sw-val :lambda-list '(m))
(cl:defmethod os_sw-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:os_sw-val is deprecated.  Use autopilot_bridge-srv:os_sw instead.")
  (os_sw m))

(cl:ensure-generic-function 'board-val :lambda-list '(m))
(cl:defmethod board-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:board-val is deprecated.  Use autopilot_bridge-srv:board instead.")
  (board m))

(cl:ensure-generic-function 'flight_custom-val :lambda-list '(m))
(cl:defmethod flight_custom-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:flight_custom-val is deprecated.  Use autopilot_bridge-srv:flight_custom instead.")
  (flight_custom m))

(cl:ensure-generic-function 'middleware_custom-val :lambda-list '(m))
(cl:defmethod middleware_custom-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:middleware_custom-val is deprecated.  Use autopilot_bridge-srv:middleware_custom instead.")
  (middleware_custom m))

(cl:ensure-generic-function 'os_custom-val :lambda-list '(m))
(cl:defmethod os_custom-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:os_custom-val is deprecated.  Use autopilot_bridge-srv:os_custom instead.")
  (os_custom m))

(cl:ensure-generic-function 'vendor_id-val :lambda-list '(m))
(cl:defmethod vendor_id-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:vendor_id-val is deprecated.  Use autopilot_bridge-srv:vendor_id instead.")
  (vendor_id m))

(cl:ensure-generic-function 'product_id-val :lambda-list '(m))
(cl:defmethod product_id-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:product_id-val is deprecated.  Use autopilot_bridge-srv:product_id instead.")
  (product_id m))

(cl:ensure-generic-function 'uid-val :lambda-list '(m))
(cl:defmethod uid-val ((m <Version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:uid-val is deprecated.  Use autopilot_bridge-srv:uid instead.")
  (uid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Version-response>) ostream)
  "Serializes a message object of type '<Version-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_mission_float) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_param_float) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_mission_int) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_command_int) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_param_union) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_ftp) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_set_attitude) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_set_position_local) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_set_position_global) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_terrain) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cap_set_actuator) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flight_sw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flight_sw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flight_sw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flight_sw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'middleware)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'middleware)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'middleware)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'middleware)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'os_sw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'os_sw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'os_sw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'os_sw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'board)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'board)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'board)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'board)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'flight_custom))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'flight_custom))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'middleware_custom))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'middleware_custom))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'os_custom))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'os_custom))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vendor_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vendor_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'product_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'product_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'uid)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Version-response>) istream)
  "Deserializes a message object of type '<Version-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_mission_float) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_param_float) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_mission_int) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_command_int) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_param_union) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_ftp) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_set_attitude) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_set_position_local) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_set_position_global) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_terrain) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cap_set_actuator) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flight_sw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flight_sw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flight_sw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flight_sw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'middleware)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'middleware)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'middleware)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'middleware)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'os_sw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'os_sw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'os_sw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'os_sw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'board)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'board)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'board)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'board)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flight_custom) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'flight_custom) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'middleware_custom) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'middleware_custom) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'os_custom) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'os_custom) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vendor_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'vendor_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'product_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'product_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'uid)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Version-response>)))
  "Returns string type for a service object of type '<Version-response>"
  "autopilot_bridge/VersionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Version-response)))
  "Returns string type for a service object of type 'Version-response"
  "autopilot_bridge/VersionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Version-response>)))
  "Returns md5sum for a message object of type '<Version-response>"
  "d45d1d0a68288be9b762029a29740e01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Version-response)))
  "Returns md5sum for a message object of type 'Version-response"
  "d45d1d0a68288be9b762029a29740e01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Version-response>)))
  "Returns full string definition for message of type '<Version-response>"
  (cl:format cl:nil "bool ok~%bool cap_mission_float~%bool cap_param_float~%bool cap_mission_int~%bool cap_command_int~%bool cap_param_union~%bool cap_ftp~%bool cap_set_attitude~%bool cap_set_position_local~%bool cap_set_position_global~%bool cap_terrain~%bool cap_set_actuator~%uint32 flight_sw~%uint32 middleware~%uint32 os_sw~%uint32 board~%string flight_custom~%string middleware_custom~%string os_custom~%uint16 vendor_id~%uint16 product_id~%uint64 uid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Version-response)))
  "Returns full string definition for message of type 'Version-response"
  (cl:format cl:nil "bool ok~%bool cap_mission_float~%bool cap_param_float~%bool cap_mission_int~%bool cap_command_int~%bool cap_param_union~%bool cap_ftp~%bool cap_set_attitude~%bool cap_set_position_local~%bool cap_set_position_global~%bool cap_terrain~%bool cap_set_actuator~%uint32 flight_sw~%uint32 middleware~%uint32 os_sw~%uint32 board~%string flight_custom~%string middleware_custom~%string os_custom~%uint16 vendor_id~%uint16 product_id~%uint64 uid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Version-response>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'flight_custom))
     4 (cl:length (cl:slot-value msg 'middleware_custom))
     4 (cl:length (cl:slot-value msg 'os_custom))
     2
     2
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Version-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Version-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':cap_mission_float (cap_mission_float msg))
    (cl:cons ':cap_param_float (cap_param_float msg))
    (cl:cons ':cap_mission_int (cap_mission_int msg))
    (cl:cons ':cap_command_int (cap_command_int msg))
    (cl:cons ':cap_param_union (cap_param_union msg))
    (cl:cons ':cap_ftp (cap_ftp msg))
    (cl:cons ':cap_set_attitude (cap_set_attitude msg))
    (cl:cons ':cap_set_position_local (cap_set_position_local msg))
    (cl:cons ':cap_set_position_global (cap_set_position_global msg))
    (cl:cons ':cap_terrain (cap_terrain msg))
    (cl:cons ':cap_set_actuator (cap_set_actuator msg))
    (cl:cons ':flight_sw (flight_sw msg))
    (cl:cons ':middleware (middleware msg))
    (cl:cons ':os_sw (os_sw msg))
    (cl:cons ':board (board msg))
    (cl:cons ':flight_custom (flight_custom msg))
    (cl:cons ':middleware_custom (middleware_custom msg))
    (cl:cons ':os_custom (os_custom msg))
    (cl:cons ':vendor_id (vendor_id msg))
    (cl:cons ':product_id (product_id msg))
    (cl:cons ':uid (uid msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Version)))
  'Version-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Version)))
  'Version-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Version)))
  "Returns string type for a service object of type '<Version>"
  "autopilot_bridge/Version")