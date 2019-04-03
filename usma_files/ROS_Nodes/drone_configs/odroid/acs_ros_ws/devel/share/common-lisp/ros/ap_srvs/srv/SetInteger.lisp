; Auto-generated. Do not edit!


(cl:in-package ap_srvs-srv)


;//! \htmlinclude SetInteger-request.msg.html

(cl:defclass <SetInteger-request> (roslisp-msg-protocol:ros-message)
  ((setting
    :reader setting
    :initarg :setting
    :type cl:integer
    :initform 0))
)

(cl:defclass SetInteger-request (<SetInteger-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInteger-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInteger-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_srvs-srv:<SetInteger-request> is deprecated: use ap_srvs-srv:SetInteger-request instead.")))

(cl:ensure-generic-function 'setting-val :lambda-list '(m))
(cl:defmethod setting-val ((m <SetInteger-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:setting-val is deprecated.  Use ap_srvs-srv:setting instead.")
  (setting m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInteger-request>) ostream)
  "Serializes a message object of type '<SetInteger-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'setting)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'setting)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'setting)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'setting)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInteger-request>) istream)
  "Deserializes a message object of type '<SetInteger-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'setting)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'setting)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'setting)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'setting)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInteger-request>)))
  "Returns string type for a service object of type '<SetInteger-request>"
  "ap_srvs/SetIntegerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInteger-request)))
  "Returns string type for a service object of type 'SetInteger-request"
  "ap_srvs/SetIntegerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInteger-request>)))
  "Returns md5sum for a message object of type '<SetInteger-request>"
  "9cc7b8927f792e461dca9cd6251d1d23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInteger-request)))
  "Returns md5sum for a message object of type 'SetInteger-request"
  "9cc7b8927f792e461dca9cd6251d1d23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInteger-request>)))
  "Returns full string definition for message of type '<SetInteger-request>"
  (cl:format cl:nil "uint32 setting~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInteger-request)))
  "Returns full string definition for message of type 'SetInteger-request"
  (cl:format cl:nil "uint32 setting~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInteger-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInteger-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInteger-request
    (cl:cons ':setting (setting msg))
))
;//! \htmlinclude SetInteger-response.msg.html

(cl:defclass <SetInteger-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetInteger-response (<SetInteger-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetInteger-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetInteger-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_srvs-srv:<SetInteger-response> is deprecated: use ap_srvs-srv:SetInteger-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetInteger-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:result-val is deprecated.  Use ap_srvs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetInteger-response>) ostream)
  "Serializes a message object of type '<SetInteger-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetInteger-response>) istream)
  "Deserializes a message object of type '<SetInteger-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetInteger-response>)))
  "Returns string type for a service object of type '<SetInteger-response>"
  "ap_srvs/SetIntegerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInteger-response)))
  "Returns string type for a service object of type 'SetInteger-response"
  "ap_srvs/SetIntegerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetInteger-response>)))
  "Returns md5sum for a message object of type '<SetInteger-response>"
  "9cc7b8927f792e461dca9cd6251d1d23")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetInteger-response)))
  "Returns md5sum for a message object of type 'SetInteger-response"
  "9cc7b8927f792e461dca9cd6251d1d23")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetInteger-response>)))
  "Returns full string definition for message of type '<SetInteger-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetInteger-response)))
  "Returns full string definition for message of type 'SetInteger-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetInteger-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetInteger-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetInteger-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetInteger)))
  'SetInteger-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetInteger)))
  'SetInteger-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetInteger)))
  "Returns string type for a service object of type '<SetInteger>"
  "ap_srvs/SetInteger")