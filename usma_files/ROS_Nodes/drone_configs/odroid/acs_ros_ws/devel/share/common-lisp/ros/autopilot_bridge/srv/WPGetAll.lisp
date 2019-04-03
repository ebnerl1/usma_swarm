; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude WPGetAll-request.msg.html

(cl:defclass <WPGetAll-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WPGetAll-request (<WPGetAll-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WPGetAll-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WPGetAll-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<WPGetAll-request> is deprecated: use autopilot_bridge-srv:WPGetAll-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WPGetAll-request>) ostream)
  "Serializes a message object of type '<WPGetAll-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WPGetAll-request>) istream)
  "Deserializes a message object of type '<WPGetAll-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WPGetAll-request>)))
  "Returns string type for a service object of type '<WPGetAll-request>"
  "autopilot_bridge/WPGetAllRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPGetAll-request)))
  "Returns string type for a service object of type 'WPGetAll-request"
  "autopilot_bridge/WPGetAllRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WPGetAll-request>)))
  "Returns md5sum for a message object of type '<WPGetAll-request>"
  "61fd937ff9a773d87567a2f8da6a06ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WPGetAll-request)))
  "Returns md5sum for a message object of type 'WPGetAll-request"
  "61fd937ff9a773d87567a2f8da6a06ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WPGetAll-request>)))
  "Returns full string definition for message of type '<WPGetAll-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WPGetAll-request)))
  "Returns full string definition for message of type 'WPGetAll-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WPGetAll-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WPGetAll-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WPGetAll-request
))
;//! \htmlinclude WPGetAll-response.msg.html

(cl:defclass <WPGetAll-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (points
    :reader points
    :initarg :points
    :type (cl:vector autopilot_bridge-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:Waypoint :initial-element (cl:make-instance 'autopilot_bridge-msg:Waypoint))))
)

(cl:defclass WPGetAll-response (<WPGetAll-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WPGetAll-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WPGetAll-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<WPGetAll-response> is deprecated: use autopilot_bridge-srv:WPGetAll-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <WPGetAll-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <WPGetAll-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:points-val is deprecated.  Use autopilot_bridge-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WPGetAll-response>) ostream)
  "Serializes a message object of type '<WPGetAll-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WPGetAll-response>) istream)
  "Deserializes a message object of type '<WPGetAll-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'autopilot_bridge-msg:Waypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WPGetAll-response>)))
  "Returns string type for a service object of type '<WPGetAll-response>"
  "autopilot_bridge/WPGetAllResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPGetAll-response)))
  "Returns string type for a service object of type 'WPGetAll-response"
  "autopilot_bridge/WPGetAllResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WPGetAll-response>)))
  "Returns md5sum for a message object of type '<WPGetAll-response>"
  "61fd937ff9a773d87567a2f8da6a06ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WPGetAll-response)))
  "Returns md5sum for a message object of type 'WPGetAll-response"
  "61fd937ff9a773d87567a2f8da6a06ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WPGetAll-response>)))
  "Returns full string definition for message of type '<WPGetAll-response>"
  (cl:format cl:nil "bool ok~%Waypoint[] points~%~%~%================================================================================~%MSG: autopilot_bridge/Waypoint~%uint16 seq~%uint8 frame~%uint16 command~%bool current~%bool autocontinue~%float64 param1~%float64 param2~%float64 param3~%float64 param4~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WPGetAll-response)))
  "Returns full string definition for message of type 'WPGetAll-response"
  (cl:format cl:nil "bool ok~%Waypoint[] points~%~%~%================================================================================~%MSG: autopilot_bridge/Waypoint~%uint16 seq~%uint8 frame~%uint16 command~%bool current~%bool autocontinue~%float64 param1~%float64 param2~%float64 param3~%float64 param4~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WPGetAll-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WPGetAll-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WPGetAll-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':points (points msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WPGetAll)))
  'WPGetAll-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WPGetAll)))
  'WPGetAll-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPGetAll)))
  "Returns string type for a service object of type '<WPGetAll>"
  "autopilot_bridge/WPGetAll")