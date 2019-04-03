; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude WPSetAll-request.msg.html

(cl:defclass <WPSetAll-request> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector autopilot_bridge-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:Waypoint :initial-element (cl:make-instance 'autopilot_bridge-msg:Waypoint))))
)

(cl:defclass WPSetAll-request (<WPSetAll-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WPSetAll-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WPSetAll-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<WPSetAll-request> is deprecated: use autopilot_bridge-srv:WPSetAll-request instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <WPSetAll-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:points-val is deprecated.  Use autopilot_bridge-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WPSetAll-request>) ostream)
  "Serializes a message object of type '<WPSetAll-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WPSetAll-request>) istream)
  "Deserializes a message object of type '<WPSetAll-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WPSetAll-request>)))
  "Returns string type for a service object of type '<WPSetAll-request>"
  "autopilot_bridge/WPSetAllRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPSetAll-request)))
  "Returns string type for a service object of type 'WPSetAll-request"
  "autopilot_bridge/WPSetAllRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WPSetAll-request>)))
  "Returns md5sum for a message object of type '<WPSetAll-request>"
  "ca7cf06c1f77e9786e36687dbdde93e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WPSetAll-request)))
  "Returns md5sum for a message object of type 'WPSetAll-request"
  "ca7cf06c1f77e9786e36687dbdde93e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WPSetAll-request>)))
  "Returns full string definition for message of type '<WPSetAll-request>"
  (cl:format cl:nil "Waypoint[] points~%~%================================================================================~%MSG: autopilot_bridge/Waypoint~%uint16 seq~%uint8 frame~%uint16 command~%bool current~%bool autocontinue~%float64 param1~%float64 param2~%float64 param3~%float64 param4~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WPSetAll-request)))
  "Returns full string definition for message of type 'WPSetAll-request"
  (cl:format cl:nil "Waypoint[] points~%~%================================================================================~%MSG: autopilot_bridge/Waypoint~%uint16 seq~%uint8 frame~%uint16 command~%bool current~%bool autocontinue~%float64 param1~%float64 param2~%float64 param3~%float64 param4~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WPSetAll-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WPSetAll-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WPSetAll-request
    (cl:cons ':points (points msg))
))
;//! \htmlinclude WPSetAll-response.msg.html

(cl:defclass <WPSetAll-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WPSetAll-response (<WPSetAll-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WPSetAll-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WPSetAll-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<WPSetAll-response> is deprecated: use autopilot_bridge-srv:WPSetAll-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <WPSetAll-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WPSetAll-response>) ostream)
  "Serializes a message object of type '<WPSetAll-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WPSetAll-response>) istream)
  "Deserializes a message object of type '<WPSetAll-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WPSetAll-response>)))
  "Returns string type for a service object of type '<WPSetAll-response>"
  "autopilot_bridge/WPSetAllResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPSetAll-response)))
  "Returns string type for a service object of type 'WPSetAll-response"
  "autopilot_bridge/WPSetAllResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WPSetAll-response>)))
  "Returns md5sum for a message object of type '<WPSetAll-response>"
  "ca7cf06c1f77e9786e36687dbdde93e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WPSetAll-response)))
  "Returns md5sum for a message object of type 'WPSetAll-response"
  "ca7cf06c1f77e9786e36687dbdde93e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WPSetAll-response>)))
  "Returns full string definition for message of type '<WPSetAll-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WPSetAll-response)))
  "Returns full string definition for message of type 'WPSetAll-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WPSetAll-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WPSetAll-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WPSetAll-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WPSetAll)))
  'WPSetAll-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WPSetAll)))
  'WPSetAll-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPSetAll)))
  "Returns string type for a service object of type '<WPSetAll>"
  "autopilot_bridge/WPSetAll")