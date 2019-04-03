; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude WPGetRange-request.msg.html

(cl:defclass <WPGetRange-request> (roslisp-msg-protocol:ros-message)
  ((low
    :reader low
    :initarg :low
    :type cl:fixnum
    :initform 0)
   (high
    :reader high
    :initarg :high
    :type cl:fixnum
    :initform 0))
)

(cl:defclass WPGetRange-request (<WPGetRange-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WPGetRange-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WPGetRange-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<WPGetRange-request> is deprecated: use autopilot_bridge-srv:WPGetRange-request instead.")))

(cl:ensure-generic-function 'low-val :lambda-list '(m))
(cl:defmethod low-val ((m <WPGetRange-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:low-val is deprecated.  Use autopilot_bridge-srv:low instead.")
  (low m))

(cl:ensure-generic-function 'high-val :lambda-list '(m))
(cl:defmethod high-val ((m <WPGetRange-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:high-val is deprecated.  Use autopilot_bridge-srv:high instead.")
  (high m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WPGetRange-request>) ostream)
  "Serializes a message object of type '<WPGetRange-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'low)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'low)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'high)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'high)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WPGetRange-request>) istream)
  "Deserializes a message object of type '<WPGetRange-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'low)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'low)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'high)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'high)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WPGetRange-request>)))
  "Returns string type for a service object of type '<WPGetRange-request>"
  "autopilot_bridge/WPGetRangeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPGetRange-request)))
  "Returns string type for a service object of type 'WPGetRange-request"
  "autopilot_bridge/WPGetRangeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WPGetRange-request>)))
  "Returns md5sum for a message object of type '<WPGetRange-request>"
  "a5c104ffb8d6e855f74938b0261a09c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WPGetRange-request)))
  "Returns md5sum for a message object of type 'WPGetRange-request"
  "a5c104ffb8d6e855f74938b0261a09c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WPGetRange-request>)))
  "Returns full string definition for message of type '<WPGetRange-request>"
  (cl:format cl:nil "uint16 low~%uint16 high~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WPGetRange-request)))
  "Returns full string definition for message of type 'WPGetRange-request"
  (cl:format cl:nil "uint16 low~%uint16 high~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WPGetRange-request>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WPGetRange-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WPGetRange-request
    (cl:cons ':low (low msg))
    (cl:cons ':high (high msg))
))
;//! \htmlinclude WPGetRange-response.msg.html

(cl:defclass <WPGetRange-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass WPGetRange-response (<WPGetRange-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WPGetRange-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WPGetRange-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<WPGetRange-response> is deprecated: use autopilot_bridge-srv:WPGetRange-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <WPGetRange-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <WPGetRange-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:points-val is deprecated.  Use autopilot_bridge-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WPGetRange-response>) ostream)
  "Serializes a message object of type '<WPGetRange-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WPGetRange-response>) istream)
  "Deserializes a message object of type '<WPGetRange-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WPGetRange-response>)))
  "Returns string type for a service object of type '<WPGetRange-response>"
  "autopilot_bridge/WPGetRangeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPGetRange-response)))
  "Returns string type for a service object of type 'WPGetRange-response"
  "autopilot_bridge/WPGetRangeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WPGetRange-response>)))
  "Returns md5sum for a message object of type '<WPGetRange-response>"
  "a5c104ffb8d6e855f74938b0261a09c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WPGetRange-response)))
  "Returns md5sum for a message object of type 'WPGetRange-response"
  "a5c104ffb8d6e855f74938b0261a09c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WPGetRange-response>)))
  "Returns full string definition for message of type '<WPGetRange-response>"
  (cl:format cl:nil "bool ok~%Waypoint[] points~%~%~%================================================================================~%MSG: autopilot_bridge/Waypoint~%uint16 seq~%uint8 frame~%uint16 command~%bool current~%bool autocontinue~%float64 param1~%float64 param2~%float64 param3~%float64 param4~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WPGetRange-response)))
  "Returns full string definition for message of type 'WPGetRange-response"
  (cl:format cl:nil "bool ok~%Waypoint[] points~%~%~%================================================================================~%MSG: autopilot_bridge/Waypoint~%uint16 seq~%uint8 frame~%uint16 command~%bool current~%bool autocontinue~%float64 param1~%float64 param2~%float64 param3~%float64 param4~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WPGetRange-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WPGetRange-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WPGetRange-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':points (points msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WPGetRange)))
  'WPGetRange-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WPGetRange)))
  'WPGetRange-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPGetRange)))
  "Returns string type for a service object of type '<WPGetRange>"
  "autopilot_bridge/WPGetRange")