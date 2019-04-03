; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude RallyGetAll-request.msg.html

(cl:defclass <RallyGetAll-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RallyGetAll-request (<RallyGetAll-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RallyGetAll-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RallyGetAll-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<RallyGetAll-request> is deprecated: use autopilot_bridge-srv:RallyGetAll-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RallyGetAll-request>) ostream)
  "Serializes a message object of type '<RallyGetAll-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RallyGetAll-request>) istream)
  "Deserializes a message object of type '<RallyGetAll-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RallyGetAll-request>)))
  "Returns string type for a service object of type '<RallyGetAll-request>"
  "autopilot_bridge/RallyGetAllRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RallyGetAll-request)))
  "Returns string type for a service object of type 'RallyGetAll-request"
  "autopilot_bridge/RallyGetAllRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RallyGetAll-request>)))
  "Returns md5sum for a message object of type '<RallyGetAll-request>"
  "a2828381673d0b45c532f4eea68ab4e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RallyGetAll-request)))
  "Returns md5sum for a message object of type 'RallyGetAll-request"
  "a2828381673d0b45c532f4eea68ab4e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RallyGetAll-request>)))
  "Returns full string definition for message of type '<RallyGetAll-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RallyGetAll-request)))
  "Returns full string definition for message of type 'RallyGetAll-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RallyGetAll-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RallyGetAll-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RallyGetAll-request
))
;//! \htmlinclude RallyGetAll-response.msg.html

(cl:defclass <RallyGetAll-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (points
    :reader points
    :initarg :points
    :type (cl:vector autopilot_bridge-msg:Rallypoint)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:Rallypoint :initial-element (cl:make-instance 'autopilot_bridge-msg:Rallypoint))))
)

(cl:defclass RallyGetAll-response (<RallyGetAll-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RallyGetAll-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RallyGetAll-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<RallyGetAll-response> is deprecated: use autopilot_bridge-srv:RallyGetAll-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <RallyGetAll-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <RallyGetAll-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:points-val is deprecated.  Use autopilot_bridge-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RallyGetAll-response>) ostream)
  "Serializes a message object of type '<RallyGetAll-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RallyGetAll-response>) istream)
  "Deserializes a message object of type '<RallyGetAll-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RallyGetAll-response>)))
  "Returns string type for a service object of type '<RallyGetAll-response>"
  "autopilot_bridge/RallyGetAllResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RallyGetAll-response)))
  "Returns string type for a service object of type 'RallyGetAll-response"
  "autopilot_bridge/RallyGetAllResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RallyGetAll-response>)))
  "Returns md5sum for a message object of type '<RallyGetAll-response>"
  "a2828381673d0b45c532f4eea68ab4e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RallyGetAll-response)))
  "Returns md5sum for a message object of type 'RallyGetAll-response"
  "a2828381673d0b45c532f4eea68ab4e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RallyGetAll-response>)))
  "Returns full string definition for message of type '<RallyGetAll-response>"
  (cl:format cl:nil "bool ok~%Rallypoint[] points~%~%~%================================================================================~%MSG: autopilot_bridge/Rallypoint~%float64 lat~%float64 lon~%float64 alt~%float64 break_alt~%float64 land_dir~%uint8 flags~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RallyGetAll-response)))
  "Returns full string definition for message of type 'RallyGetAll-response"
  (cl:format cl:nil "bool ok~%Rallypoint[] points~%~%~%================================================================================~%MSG: autopilot_bridge/Rallypoint~%float64 lat~%float64 lon~%float64 alt~%float64 break_alt~%float64 land_dir~%uint8 flags~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RallyGetAll-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RallyGetAll-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RallyGetAll-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':points (points msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RallyGetAll)))
  'RallyGetAll-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RallyGetAll)))
  'RallyGetAll-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RallyGetAll)))
  "Returns string type for a service object of type '<RallyGetAll>"
  "autopilot_bridge/RallyGetAll")