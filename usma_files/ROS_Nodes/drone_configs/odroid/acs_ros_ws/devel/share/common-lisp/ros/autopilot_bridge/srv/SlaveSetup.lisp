; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude SlaveSetup-request.msg.html

(cl:defclass <SlaveSetup-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil)
   (channel
    :reader channel
    :initarg :channel
    :type cl:string
    :initform ""))
)

(cl:defclass SlaveSetup-request (<SlaveSetup-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SlaveSetup-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SlaveSetup-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<SlaveSetup-request> is deprecated: use autopilot_bridge-srv:SlaveSetup-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <SlaveSetup-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:enable-val is deprecated.  Use autopilot_bridge-srv:enable instead.")
  (enable m))

(cl:ensure-generic-function 'channel-val :lambda-list '(m))
(cl:defmethod channel-val ((m <SlaveSetup-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:channel-val is deprecated.  Use autopilot_bridge-srv:channel instead.")
  (channel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SlaveSetup-request>) ostream)
  "Serializes a message object of type '<SlaveSetup-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'channel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'channel))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SlaveSetup-request>) istream)
  "Deserializes a message object of type '<SlaveSetup-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'channel) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'channel) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SlaveSetup-request>)))
  "Returns string type for a service object of type '<SlaveSetup-request>"
  "autopilot_bridge/SlaveSetupRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SlaveSetup-request)))
  "Returns string type for a service object of type 'SlaveSetup-request"
  "autopilot_bridge/SlaveSetupRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SlaveSetup-request>)))
  "Returns md5sum for a message object of type '<SlaveSetup-request>"
  "d5924e2477b62d263f4cdf77d9e962e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SlaveSetup-request)))
  "Returns md5sum for a message object of type 'SlaveSetup-request"
  "d5924e2477b62d263f4cdf77d9e962e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SlaveSetup-request>)))
  "Returns full string definition for message of type '<SlaveSetup-request>"
  (cl:format cl:nil "bool enable~%string channel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SlaveSetup-request)))
  "Returns full string definition for message of type 'SlaveSetup-request"
  (cl:format cl:nil "bool enable~%string channel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SlaveSetup-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'channel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SlaveSetup-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SlaveSetup-request
    (cl:cons ':enable (enable msg))
    (cl:cons ':channel (channel msg))
))
;//! \htmlinclude SlaveSetup-response.msg.html

(cl:defclass <SlaveSetup-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SlaveSetup-response (<SlaveSetup-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SlaveSetup-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SlaveSetup-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<SlaveSetup-response> is deprecated: use autopilot_bridge-srv:SlaveSetup-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SlaveSetup-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SlaveSetup-response>) ostream)
  "Serializes a message object of type '<SlaveSetup-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SlaveSetup-response>) istream)
  "Deserializes a message object of type '<SlaveSetup-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SlaveSetup-response>)))
  "Returns string type for a service object of type '<SlaveSetup-response>"
  "autopilot_bridge/SlaveSetupResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SlaveSetup-response)))
  "Returns string type for a service object of type 'SlaveSetup-response"
  "autopilot_bridge/SlaveSetupResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SlaveSetup-response>)))
  "Returns md5sum for a message object of type '<SlaveSetup-response>"
  "d5924e2477b62d263f4cdf77d9e962e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SlaveSetup-response)))
  "Returns md5sum for a message object of type 'SlaveSetup-response"
  "d5924e2477b62d263f4cdf77d9e962e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SlaveSetup-response>)))
  "Returns full string definition for message of type '<SlaveSetup-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SlaveSetup-response)))
  "Returns full string definition for message of type 'SlaveSetup-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SlaveSetup-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SlaveSetup-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SlaveSetup-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SlaveSetup)))
  'SlaveSetup-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SlaveSetup)))
  'SlaveSetup-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SlaveSetup)))
  "Returns string type for a service object of type '<SlaveSetup>"
  "autopilot_bridge/SlaveSetup")