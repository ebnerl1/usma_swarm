; Auto-generated. Do not edit!


(cl:in-package ap_srvs-srv)


;//! \htmlinclude SetBehavior-request.msg.html

(cl:defclass <SetBehavior-request> (roslisp-msg-protocol:ros-message)
  ((params
    :reader params
    :initarg :params
    :type ap_msgs-msg:BehaviorParameters
    :initform (cl:make-instance 'ap_msgs-msg:BehaviorParameters)))
)

(cl:defclass SetBehavior-request (<SetBehavior-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBehavior-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBehavior-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_srvs-srv:<SetBehavior-request> is deprecated: use ap_srvs-srv:SetBehavior-request instead.")))

(cl:ensure-generic-function 'params-val :lambda-list '(m))
(cl:defmethod params-val ((m <SetBehavior-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:params-val is deprecated.  Use ap_srvs-srv:params instead.")
  (params m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBehavior-request>) ostream)
  "Serializes a message object of type '<SetBehavior-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'params) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBehavior-request>) istream)
  "Deserializes a message object of type '<SetBehavior-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'params) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBehavior-request>)))
  "Returns string type for a service object of type '<SetBehavior-request>"
  "ap_srvs/SetBehaviorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBehavior-request)))
  "Returns string type for a service object of type 'SetBehavior-request"
  "ap_srvs/SetBehaviorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBehavior-request>)))
  "Returns md5sum for a message object of type '<SetBehavior-request>"
  "a69d05b7cab2cafea64d5e3ae6bea710")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBehavior-request)))
  "Returns md5sum for a message object of type 'SetBehavior-request"
  "a69d05b7cab2cafea64d5e3ae6bea710")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBehavior-request>)))
  "Returns full string definition for message of type '<SetBehavior-request>"
  (cl:format cl:nil "ap_msgs/BehaviorParameters params~%~%================================================================================~%MSG: ap_msgs/BehaviorParameters~%uint8 id~%uint8[] params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBehavior-request)))
  "Returns full string definition for message of type 'SetBehavior-request"
  (cl:format cl:nil "ap_msgs/BehaviorParameters params~%~%================================================================================~%MSG: ap_msgs/BehaviorParameters~%uint8 id~%uint8[] params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBehavior-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'params))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBehavior-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBehavior-request
    (cl:cons ':params (params msg))
))
;//! \htmlinclude SetBehavior-response.msg.html

(cl:defclass <SetBehavior-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetBehavior-response (<SetBehavior-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBehavior-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBehavior-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_srvs-srv:<SetBehavior-response> is deprecated: use ap_srvs-srv:SetBehavior-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetBehavior-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_srvs-srv:result-val is deprecated.  Use ap_srvs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBehavior-response>) ostream)
  "Serializes a message object of type '<SetBehavior-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBehavior-response>) istream)
  "Deserializes a message object of type '<SetBehavior-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBehavior-response>)))
  "Returns string type for a service object of type '<SetBehavior-response>"
  "ap_srvs/SetBehaviorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBehavior-response)))
  "Returns string type for a service object of type 'SetBehavior-response"
  "ap_srvs/SetBehaviorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBehavior-response>)))
  "Returns md5sum for a message object of type '<SetBehavior-response>"
  "a69d05b7cab2cafea64d5e3ae6bea710")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBehavior-response)))
  "Returns md5sum for a message object of type 'SetBehavior-response"
  "a69d05b7cab2cafea64d5e3ae6bea710")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBehavior-response>)))
  "Returns full string definition for message of type '<SetBehavior-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBehavior-response)))
  "Returns full string definition for message of type 'SetBehavior-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBehavior-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBehavior-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBehavior-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetBehavior)))
  'SetBehavior-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetBehavior)))
  'SetBehavior-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBehavior)))
  "Returns string type for a service object of type '<SetBehavior>"
  "ap_srvs/SetBehavior")