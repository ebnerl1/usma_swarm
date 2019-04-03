; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-srv)


;//! \htmlinclude FenceGetAll-request.msg.html

(cl:defclass <FenceGetAll-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FenceGetAll-request (<FenceGetAll-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FenceGetAll-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FenceGetAll-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<FenceGetAll-request> is deprecated: use autopilot_bridge-srv:FenceGetAll-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FenceGetAll-request>) ostream)
  "Serializes a message object of type '<FenceGetAll-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FenceGetAll-request>) istream)
  "Deserializes a message object of type '<FenceGetAll-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FenceGetAll-request>)))
  "Returns string type for a service object of type '<FenceGetAll-request>"
  "autopilot_bridge/FenceGetAllRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FenceGetAll-request)))
  "Returns string type for a service object of type 'FenceGetAll-request"
  "autopilot_bridge/FenceGetAllRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FenceGetAll-request>)))
  "Returns md5sum for a message object of type '<FenceGetAll-request>"
  "3af4de476ae820ab7043ee91d5577ce1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FenceGetAll-request)))
  "Returns md5sum for a message object of type 'FenceGetAll-request"
  "3af4de476ae820ab7043ee91d5577ce1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FenceGetAll-request>)))
  "Returns full string definition for message of type '<FenceGetAll-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FenceGetAll-request)))
  "Returns full string definition for message of type 'FenceGetAll-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FenceGetAll-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FenceGetAll-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FenceGetAll-request
))
;//! \htmlinclude FenceGetAll-response.msg.html

(cl:defclass <FenceGetAll-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (points
    :reader points
    :initarg :points
    :type (cl:vector autopilot_bridge-msg:Fencepoint)
   :initform (cl:make-array 0 :element-type 'autopilot_bridge-msg:Fencepoint :initial-element (cl:make-instance 'autopilot_bridge-msg:Fencepoint))))
)

(cl:defclass FenceGetAll-response (<FenceGetAll-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FenceGetAll-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FenceGetAll-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-srv:<FenceGetAll-response> is deprecated: use autopilot_bridge-srv:FenceGetAll-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <FenceGetAll-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:ok-val is deprecated.  Use autopilot_bridge-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <FenceGetAll-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-srv:points-val is deprecated.  Use autopilot_bridge-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FenceGetAll-response>) ostream)
  "Serializes a message object of type '<FenceGetAll-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FenceGetAll-response>) istream)
  "Deserializes a message object of type '<FenceGetAll-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FenceGetAll-response>)))
  "Returns string type for a service object of type '<FenceGetAll-response>"
  "autopilot_bridge/FenceGetAllResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FenceGetAll-response)))
  "Returns string type for a service object of type 'FenceGetAll-response"
  "autopilot_bridge/FenceGetAllResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FenceGetAll-response>)))
  "Returns md5sum for a message object of type '<FenceGetAll-response>"
  "3af4de476ae820ab7043ee91d5577ce1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FenceGetAll-response)))
  "Returns md5sum for a message object of type 'FenceGetAll-response"
  "3af4de476ae820ab7043ee91d5577ce1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FenceGetAll-response>)))
  "Returns full string definition for message of type '<FenceGetAll-response>"
  (cl:format cl:nil "bool ok~%Fencepoint[] points~%~%~%================================================================================~%MSG: autopilot_bridge/Fencepoint~%float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FenceGetAll-response)))
  "Returns full string definition for message of type 'FenceGetAll-response"
  (cl:format cl:nil "bool ok~%Fencepoint[] points~%~%~%================================================================================~%MSG: autopilot_bridge/Fencepoint~%float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FenceGetAll-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FenceGetAll-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FenceGetAll-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':points (points msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FenceGetAll)))
  'FenceGetAll-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FenceGetAll)))
  'FenceGetAll-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FenceGetAll)))
  "Returns string type for a service object of type '<FenceGetAll>"
  "autopilot_bridge/FenceGetAll")