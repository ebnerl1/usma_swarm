; Auto-generated. Do not edit!


(cl:in-package ap_srvs-srv)


;//! \htmlinclude RegisterBehavior-request.msg.html

(cl:defclass <RegisterBehavior-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (wpt_ctrl
    :reader wpt_ctrl
    :initarg :wpt_ctrl
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RegisterBehavior-request (<RegisterBehavior-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterBehavior-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterBehavior-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_srvs-srv:<RegisterBehavior-request> is deprecated: use ap_srvs-srv:RegisterBehavior-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <RegisterBehavior-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:id-val is deprecated.  Use ap_srvs-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <RegisterBehavior-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:name-val is deprecated.  Use ap_srvs-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'wpt_ctrl-val :lambda-list '(m))
(cl:defmethod wpt_ctrl-val ((m <RegisterBehavior-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:wpt_ctrl-val is deprecated.  Use ap_srvs-srv:wpt_ctrl instead.")
  (wpt_ctrl m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterBehavior-request>) ostream)
  "Serializes a message object of type '<RegisterBehavior-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wpt_ctrl) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterBehavior-request>) istream)
  "Deserializes a message object of type '<RegisterBehavior-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'wpt_ctrl) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterBehavior-request>)))
  "Returns string type for a service object of type '<RegisterBehavior-request>"
  "ap_srvs/RegisterBehaviorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterBehavior-request)))
  "Returns string type for a service object of type 'RegisterBehavior-request"
  "ap_srvs/RegisterBehaviorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterBehavior-request>)))
  "Returns md5sum for a message object of type '<RegisterBehavior-request>"
  "1e7cf8b044a2fb2667fbc9a807ca43da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterBehavior-request)))
  "Returns md5sum for a message object of type 'RegisterBehavior-request"
  "1e7cf8b044a2fb2667fbc9a807ca43da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterBehavior-request>)))
  "Returns full string definition for message of type '<RegisterBehavior-request>"
  (cl:format cl:nil "uint8 id~%string name~%bool wpt_ctrl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterBehavior-request)))
  "Returns full string definition for message of type 'RegisterBehavior-request"
  (cl:format cl:nil "uint8 id~%string name~%bool wpt_ctrl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterBehavior-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterBehavior-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterBehavior-request
    (cl:cons ':id (id msg))
    (cl:cons ':name (name msg))
    (cl:cons ':wpt_ctrl (wpt_ctrl msg))
))
;//! \htmlinclude RegisterBehavior-response.msg.html

(cl:defclass <RegisterBehavior-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RegisterBehavior-response (<RegisterBehavior-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterBehavior-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterBehavior-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_srvs-srv:<RegisterBehavior-response> is deprecated: use ap_srvs-srv:RegisterBehavior-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <RegisterBehavior-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:result-val is deprecated.  Use ap_srvs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterBehavior-response>) ostream)
  "Serializes a message object of type '<RegisterBehavior-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterBehavior-response>) istream)
  "Deserializes a message object of type '<RegisterBehavior-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterBehavior-response>)))
  "Returns string type for a service object of type '<RegisterBehavior-response>"
  "ap_srvs/RegisterBehaviorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterBehavior-response)))
  "Returns string type for a service object of type 'RegisterBehavior-response"
  "ap_srvs/RegisterBehaviorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterBehavior-response>)))
  "Returns md5sum for a message object of type '<RegisterBehavior-response>"
  "1e7cf8b044a2fb2667fbc9a807ca43da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterBehavior-response)))
  "Returns md5sum for a message object of type 'RegisterBehavior-response"
  "1e7cf8b044a2fb2667fbc9a807ca43da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterBehavior-response>)))
  "Returns full string definition for message of type '<RegisterBehavior-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterBehavior-response)))
  "Returns full string definition for message of type 'RegisterBehavior-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterBehavior-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterBehavior-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterBehavior-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RegisterBehavior)))
  'RegisterBehavior-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RegisterBehavior)))
  'RegisterBehavior-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterBehavior)))
  "Returns string type for a service object of type '<RegisterBehavior>"
  "ap_srvs/RegisterBehavior")