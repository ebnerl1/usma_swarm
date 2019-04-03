; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude FenceSetAll-request.msg.html

(cl:defclass <FenceSetAll-request> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector autopilot_bridge-msg:Fencepoint)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:Fencepoint :initial-element (cl:make-instance 'autopilot_bridge-msg:Fencepoint))))
)

(cl:defclass FenceSetAll-request (<FenceSetAll-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FenceSetAll-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FenceSetAll-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<FenceSetAll-request> is deprecated: use autopilot_bridge-srv:FenceSetAll-request instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <FenceSetAll-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:points-val is deprecated.  Use autopilot_bridge-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FenceSetAll-request>) ostream)
  "Serializes a message object of type '<FenceSetAll-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FenceSetAll-request>) istream)
  "Deserializes a message object of type '<FenceSetAll-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'autopilot_bridge-msg:Fencepoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FenceSetAll-request>)))
  "Returns string type for a service object of type '<FenceSetAll-request>"
  "autopilot_bridge/FenceSetAllRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FenceSetAll-request)))
  "Returns string type for a service object of type 'FenceSetAll-request"
  "autopilot_bridge/FenceSetAllRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FenceSetAll-request>)))
  "Returns md5sum for a message object of type '<FenceSetAll-request>"
  "f88ef42c3c089842e6bb8c2306bcba9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FenceSetAll-request)))
  "Returns md5sum for a message object of type 'FenceSetAll-request"
  "f88ef42c3c089842e6bb8c2306bcba9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FenceSetAll-request>)))
  "Returns full string definition for message of type '<FenceSetAll-request>"
  (cl:format cl:nil "Fencepoint[] points~%~%================================================================================~%MSG: autopilot_bridge/Fencepoint~%float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FenceSetAll-request)))
  "Returns full string definition for message of type 'FenceSetAll-request"
  (cl:format cl:nil "Fencepoint[] points~%~%================================================================================~%MSG: autopilot_bridge/Fencepoint~%float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FenceSetAll-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FenceSetAll-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FenceSetAll-request
    (cl:cons ':points (points msg))
))
;//! \htmlinclude FenceSetAll-response.msg.html

(cl:defclass <FenceSetAll-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FenceSetAll-response (<FenceSetAll-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FenceSetAll-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FenceSetAll-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<FenceSetAll-response> is deprecated: use autopilot_bridge-srv:FenceSetAll-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <FenceSetAll-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FenceSetAll-response>) ostream)
  "Serializes a message object of type '<FenceSetAll-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FenceSetAll-response>) istream)
  "Deserializes a message object of type '<FenceSetAll-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FenceSetAll-response>)))
  "Returns string type for a service object of type '<FenceSetAll-response>"
  "autopilot_bridge/FenceSetAllResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FenceSetAll-response)))
  "Returns string type for a service object of type 'FenceSetAll-response"
  "autopilot_bridge/FenceSetAllResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FenceSetAll-response>)))
  "Returns md5sum for a message object of type '<FenceSetAll-response>"
  "f88ef42c3c089842e6bb8c2306bcba9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FenceSetAll-response)))
  "Returns md5sum for a message object of type 'FenceSetAll-response"
  "f88ef42c3c089842e6bb8c2306bcba9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FenceSetAll-response>)))
  "Returns full string definition for message of type '<FenceSetAll-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FenceSetAll-response)))
  "Returns full string definition for message of type 'FenceSetAll-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FenceSetAll-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FenceSetAll-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FenceSetAll-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FenceSetAll)))
  'FenceSetAll-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FenceSetAll)))
  'FenceSetAll-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FenceSetAll)))
  "Returns string type for a service object of type '<FenceSetAll>"
  "autopilot_bridge/FenceSetAll")