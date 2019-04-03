; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude RallySetAll-request.msg.html

(cl:defclass <RallySetAll-request> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector autopilot_bridge-msg:Rallypoint)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:Rallypoint :initial-element (cl:make-instance 'autopilot_bridge-msg:Rallypoint))))
)

(cl:defclass RallySetAll-request (<RallySetAll-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RallySetAll-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RallySetAll-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<RallySetAll-request> is deprecated: use autopilot_bridge-srv:RallySetAll-request instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <RallySetAll-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:points-val is deprecated.  Use autopilot_bridge-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RallySetAll-request>) ostream)
  "Serializes a message object of type '<RallySetAll-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RallySetAll-request>) istream)
  "Deserializes a message object of type '<RallySetAll-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'autopilot_bridge-msg:Rallypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RallySetAll-request>)))
  "Returns string type for a service object of type '<RallySetAll-request>"
  "autopilot_bridge/RallySetAllRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RallySetAll-request)))
  "Returns string type for a service object of type 'RallySetAll-request"
  "autopilot_bridge/RallySetAllRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RallySetAll-request>)))
  "Returns md5sum for a message object of type '<RallySetAll-request>"
  "3b61b953a20eeadfbf79e0c55e54fdea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RallySetAll-request)))
  "Returns md5sum for a message object of type 'RallySetAll-request"
  "3b61b953a20eeadfbf79e0c55e54fdea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RallySetAll-request>)))
  "Returns full string definition for message of type '<RallySetAll-request>"
  (cl:format cl:nil "Rallypoint[] points~%~%================================================================================~%MSG: autopilot_bridge/Rallypoint~%float64 lat~%float64 lon~%float64 alt~%float64 break_alt~%float64 land_dir~%uint8 flags~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RallySetAll-request)))
  "Returns full string definition for message of type 'RallySetAll-request"
  (cl:format cl:nil "Rallypoint[] points~%~%================================================================================~%MSG: autopilot_bridge/Rallypoint~%float64 lat~%float64 lon~%float64 alt~%float64 break_alt~%float64 land_dir~%uint8 flags~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RallySetAll-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RallySetAll-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RallySetAll-request
    (cl:cons ':points (points msg))
))
;//! \htmlinclude RallySetAll-response.msg.html

(cl:defclass <RallySetAll-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RallySetAll-response (<RallySetAll-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RallySetAll-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RallySetAll-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<RallySetAll-response> is deprecated: use autopilot_bridge-srv:RallySetAll-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <RallySetAll-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RallySetAll-response>) ostream)
  "Serializes a message object of type '<RallySetAll-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RallySetAll-response>) istream)
  "Deserializes a message object of type '<RallySetAll-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RallySetAll-response>)))
  "Returns string type for a service object of type '<RallySetAll-response>"
  "autopilot_bridge/RallySetAllResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RallySetAll-response)))
  "Returns string type for a service object of type 'RallySetAll-response"
  "autopilot_bridge/RallySetAllResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RallySetAll-response>)))
  "Returns md5sum for a message object of type '<RallySetAll-response>"
  "3b61b953a20eeadfbf79e0c55e54fdea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RallySetAll-response)))
  "Returns md5sum for a message object of type 'RallySetAll-response"
  "3b61b953a20eeadfbf79e0c55e54fdea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RallySetAll-response>)))
  "Returns full string definition for message of type '<RallySetAll-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RallySetAll-response)))
  "Returns full string definition for message of type 'RallySetAll-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RallySetAll-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RallySetAll-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RallySetAll-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RallySetAll)))
  'RallySetAll-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RallySetAll)))
  'RallySetAll-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RallySetAll)))
  "Returns string type for a service object of type '<RallySetAll>"
  "autopilot_bridge/RallySetAll")