; Auto-generated. Do not edit!


(cl:in-package ap_srvs-srv)


;//! \htmlinclude SetBoolean-request.msg.html

(cl:defclass <SetBoolean-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetBoolean-request (<SetBoolean-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBoolean-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBoolean-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_srvs-srv:<SetBoolean-request> is deprecated: use ap_srvs-srv:SetBoolean-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <SetBoolean-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:enable-val is deprecated.  Use ap_srvs-srv:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBoolean-request>) ostream)
  "Serializes a message object of type '<SetBoolean-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBoolean-request>) istream)
  "Deserializes a message object of type '<SetBoolean-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBoolean-request>)))
  "Returns string type for a service object of type '<SetBoolean-request>"
  "ap_srvs/SetBooleanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBoolean-request)))
  "Returns string type for a service object of type 'SetBoolean-request"
  "ap_srvs/SetBooleanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBoolean-request>)))
  "Returns md5sum for a message object of type '<SetBoolean-request>"
  "29d58f387352c15c4e4f5763022ae875")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBoolean-request)))
  "Returns md5sum for a message object of type 'SetBoolean-request"
  "29d58f387352c15c4e4f5763022ae875")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBoolean-request>)))
  "Returns full string definition for message of type '<SetBoolean-request>"
  (cl:format cl:nil "bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBoolean-request)))
  "Returns full string definition for message of type 'SetBoolean-request"
  (cl:format cl:nil "bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBoolean-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBoolean-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBoolean-request
    (cl:cons ':enable (enable msg))
))
;//! \htmlinclude SetBoolean-response.msg.html

(cl:defclass <SetBoolean-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetBoolean-response (<SetBoolean-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBoolean-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBoolean-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_srvs-srv:<SetBoolean-response> is deprecated: use ap_srvs-srv:SetBoolean-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetBoolean-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:result-val is deprecated.  Use ap_srvs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBoolean-response>) ostream)
  "Serializes a message object of type '<SetBoolean-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBoolean-response>) istream)
  "Deserializes a message object of type '<SetBoolean-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBoolean-response>)))
  "Returns string type for a service object of type '<SetBoolean-response>"
  "ap_srvs/SetBooleanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBoolean-response)))
  "Returns string type for a service object of type 'SetBoolean-response"
  "ap_srvs/SetBooleanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBoolean-response>)))
  "Returns md5sum for a message object of type '<SetBoolean-response>"
  "29d58f387352c15c4e4f5763022ae875")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBoolean-response)))
  "Returns md5sum for a message object of type 'SetBoolean-response"
  "29d58f387352c15c4e4f5763022ae875")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBoolean-response>)))
  "Returns full string definition for message of type '<SetBoolean-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBoolean-response)))
  "Returns full string definition for message of type 'SetBoolean-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBoolean-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBoolean-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBoolean-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetBoolean)))
  'SetBoolean-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetBoolean)))
  'SetBoolean-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBoolean)))
  "Returns string type for a service object of type '<SetBoolean>"
  "ap_srvs/SetBoolean")