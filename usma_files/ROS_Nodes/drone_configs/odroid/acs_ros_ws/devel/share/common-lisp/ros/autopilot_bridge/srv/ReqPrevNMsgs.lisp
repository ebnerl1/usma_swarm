; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude ReqPrevNMsgs-request.msg.html

(cl:defclass <ReqPrevNMsgs-request> (roslisp-msg-protocol:ros-message)
  ((n
    :reader n
    :initarg :n
    :type cl:fixnum
    :initform 0)
   (since_seq
    :reader since_seq
    :initarg :since_seq
    :type cl:integer
    :initform 0))
)

(cl:defclass ReqPrevNMsgs-request (<ReqPrevNMsgs-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReqPrevNMsgs-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReqPrevNMsgs-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<ReqPrevNMsgs-request> is deprecated: use autopilot_bridge-srv:ReqPrevNMsgs-request instead.")))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <ReqPrevNMsgs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:n-val is deprecated.  Use autopilot_bridge-srv:n instead.")
  (n m))

(cl:ensure-generic-function 'since_seq-val :lambda-list '(m))
(cl:defmethod since_seq-val ((m <ReqPrevNMsgs-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:since_seq-val is deprecated.  Use autopilot_bridge-srv:since_seq instead.")
  (since_seq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReqPrevNMsgs-request>) ostream)
  "Serializes a message object of type '<ReqPrevNMsgs-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'since_seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'since_seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'since_seq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'since_seq)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReqPrevNMsgs-request>) istream)
  "Deserializes a message object of type '<ReqPrevNMsgs-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'since_seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'since_seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'since_seq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'since_seq)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReqPrevNMsgs-request>)))
  "Returns string type for a service object of type '<ReqPrevNMsgs-request>"
  "autopilot_bridge/ReqPrevNMsgsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReqPrevNMsgs-request)))
  "Returns string type for a service object of type 'ReqPrevNMsgs-request"
  "autopilot_bridge/ReqPrevNMsgsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReqPrevNMsgs-request>)))
  "Returns md5sum for a message object of type '<ReqPrevNMsgs-request>"
  "5b9a3726344783db5cbe1607b04eb17e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReqPrevNMsgs-request)))
  "Returns md5sum for a message object of type 'ReqPrevNMsgs-request"
  "5b9a3726344783db5cbe1607b04eb17e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReqPrevNMsgs-request>)))
  "Returns full string definition for message of type '<ReqPrevNMsgs-request>"
  (cl:format cl:nil "uint8 n~%uint32 since_seq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReqPrevNMsgs-request)))
  "Returns full string definition for message of type 'ReqPrevNMsgs-request"
  (cl:format cl:nil "uint8 n~%uint32 since_seq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReqPrevNMsgs-request>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReqPrevNMsgs-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ReqPrevNMsgs-request
    (cl:cons ':n (n msg))
    (cl:cons ':since_seq (since_seq msg))
))
;//! \htmlinclude ReqPrevNMsgs-response.msg.html

(cl:defclass <ReqPrevNMsgs-response> (roslisp-msg-protocol:ros-message)
  ((msgs
    :reader msgs
    :initarg :msgs
    :type (cl:vector autopilot_bridge-msg:AutoPilotMsg)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:AutoPilotMsg :initial-element (cl:make-instance 'autopilot_bridge-msg:AutoPilotMsg))))
)

(cl:defclass ReqPrevNMsgs-response (<ReqPrevNMsgs-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ReqPrevNMsgs-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ReqPrevNMsgs-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<ReqPrevNMsgs-response> is deprecated: use autopilot_bridge-srv:ReqPrevNMsgs-response instead.")))

(cl:ensure-generic-function 'msgs-val :lambda-list '(m))
(cl:defmethod msgs-val ((m <ReqPrevNMsgs-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:msgs-val is deprecated.  Use autopilot_bridge-srv:msgs instead.")
  (msgs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ReqPrevNMsgs-response>) ostream)
  "Serializes a message object of type '<ReqPrevNMsgs-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'msgs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'msgs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ReqPrevNMsgs-response>) istream)
  "Deserializes a message object of type '<ReqPrevNMsgs-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'msgs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'msgs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'autopilot_bridge-msg:AutoPilotMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ReqPrevNMsgs-response>)))
  "Returns string type for a service object of type '<ReqPrevNMsgs-response>"
  "autopilot_bridge/ReqPrevNMsgsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReqPrevNMsgs-response)))
  "Returns string type for a service object of type 'ReqPrevNMsgs-response"
  "autopilot_bridge/ReqPrevNMsgsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ReqPrevNMsgs-response>)))
  "Returns md5sum for a message object of type '<ReqPrevNMsgs-response>"
  "5b9a3726344783db5cbe1607b04eb17e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ReqPrevNMsgs-response)))
  "Returns md5sum for a message object of type 'ReqPrevNMsgs-response"
  "5b9a3726344783db5cbe1607b04eb17e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ReqPrevNMsgs-response>)))
  "Returns full string definition for message of type '<ReqPrevNMsgs-response>"
  (cl:format cl:nil "AutoPilotMsg[] msgs~%~%~%================================================================================~%MSG: autopilot_bridge/AutoPilotMsg~%uint32  seq~%string  text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ReqPrevNMsgs-response)))
  "Returns full string definition for message of type 'ReqPrevNMsgs-response"
  (cl:format cl:nil "AutoPilotMsg[] msgs~%~%~%================================================================================~%MSG: autopilot_bridge/AutoPilotMsg~%uint32  seq~%string  text~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ReqPrevNMsgs-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'msgs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ReqPrevNMsgs-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ReqPrevNMsgs-response
    (cl:cons ':msgs (msgs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ReqPrevNMsgs)))
  'ReqPrevNMsgs-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ReqPrevNMsgs)))
  'ReqPrevNMsgs-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ReqPrevNMsgs)))
  "Returns string type for a service object of type '<ReqPrevNMsgs>"
  "autopilot_bridge/ReqPrevNMsgs")