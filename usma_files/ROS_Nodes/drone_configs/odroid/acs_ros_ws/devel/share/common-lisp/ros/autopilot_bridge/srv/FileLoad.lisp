; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude FileLoad-request.msg.html

(cl:defclass <FileLoad-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass FileLoad-request (<FileLoad-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FileLoad-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FileLoad-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<FileLoad-request> is deprecated: use autopilot_bridge-srv:FileLoad-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <FileLoad-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:name-val is deprecated.  Use autopilot_bridge-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FileLoad-request>) ostream)
  "Serializes a message object of type '<FileLoad-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FileLoad-request>) istream)
  "Deserializes a message object of type '<FileLoad-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FileLoad-request>)))
  "Returns string type for a service object of type '<FileLoad-request>"
  "autopilot_bridge/FileLoadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FileLoad-request)))
  "Returns string type for a service object of type 'FileLoad-request"
  "autopilot_bridge/FileLoadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FileLoad-request>)))
  "Returns md5sum for a message object of type '<FileLoad-request>"
  "647e5c54b8d6468952d8d31f1efe96c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FileLoad-request)))
  "Returns md5sum for a message object of type 'FileLoad-request"
  "647e5c54b8d6468952d8d31f1efe96c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FileLoad-request>)))
  "Returns full string definition for message of type '<FileLoad-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FileLoad-request)))
  "Returns full string definition for message of type 'FileLoad-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FileLoad-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FileLoad-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FileLoad-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude FileLoad-response.msg.html

(cl:defclass <FileLoad-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FileLoad-response (<FileLoad-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FileLoad-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FileLoad-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<FileLoad-response> is deprecated: use autopilot_bridge-srv:FileLoad-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <FileLoad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FileLoad-response>) ostream)
  "Serializes a message object of type '<FileLoad-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FileLoad-response>) istream)
  "Deserializes a message object of type '<FileLoad-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FileLoad-response>)))
  "Returns string type for a service object of type '<FileLoad-response>"
  "autopilot_bridge/FileLoadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FileLoad-response)))
  "Returns string type for a service object of type 'FileLoad-response"
  "autopilot_bridge/FileLoadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FileLoad-response>)))
  "Returns md5sum for a message object of type '<FileLoad-response>"
  "647e5c54b8d6468952d8d31f1efe96c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FileLoad-response)))
  "Returns md5sum for a message object of type 'FileLoad-response"
  "647e5c54b8d6468952d8d31f1efe96c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FileLoad-response>)))
  "Returns full string definition for message of type '<FileLoad-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FileLoad-response)))
  "Returns full string definition for message of type 'FileLoad-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FileLoad-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FileLoad-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FileLoad-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FileLoad)))
  'FileLoad-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FileLoad)))
  'FileLoad-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FileLoad)))
  "Returns string type for a service object of type '<FileLoad>"
  "autopilot_bridge/FileLoad")