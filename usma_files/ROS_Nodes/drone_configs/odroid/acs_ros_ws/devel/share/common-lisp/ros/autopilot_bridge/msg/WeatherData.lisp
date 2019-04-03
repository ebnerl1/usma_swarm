; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-msg)


;//! \htmlinclude WeatherData.msg.html

(cl:defclass <WeatherData> (roslisp-msg-protocol:ros-message)
  ((baro_millibars
    :reader baro_millibars
    :initarg :baro_millibars
    :type cl:float
    :initform 0.0)
   (temp_C
    :reader temp_C
    :initarg :temp_C
    :type cl:float
    :initform 0.0)
   (wind_mph
    :reader wind_mph
    :initarg :wind_mph
    :type cl:float
    :initform 0.0)
   (wind_direction
    :reader wind_direction
    :initarg :wind_direction
    :type cl:float
    :initform 0.0))
)

(cl:defclass WeatherData (<WeatherData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WeatherData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WeatherData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-msg:<WeatherData> is deprecated: use autopilot_bridge-msg:WeatherData instead.")))

(cl:ensure-generic-function 'baro_millibars-val :lambda-list '(m))
(cl:defmethod baro_millibars-val ((m <WeatherData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:baro_millibars-val is deprecated.  Use autopilot_bridge-msg:baro_millibars instead.")
  (baro_millibars m))

(cl:ensure-generic-function 'temp_C-val :lambda-list '(m))
(cl:defmethod temp_C-val ((m <WeatherData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:temp_C-val is deprecated.  Use autopilot_bridge-msg:temp_C instead.")
  (temp_C m))

(cl:ensure-generic-function 'wind_mph-val :lambda-list '(m))
(cl:defmethod wind_mph-val ((m <WeatherData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:wind_mph-val is deprecated.  Use autopilot_bridge-msg:wind_mph instead.")
  (wind_mph m))

(cl:ensure-generic-function 'wind_direction-val :lambda-list '(m))
(cl:defmethod wind_direction-val ((m <WeatherData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:wind_direction-val is deprecated.  Use autopilot_bridge-msg:wind_direction instead.")
  (wind_direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WeatherData>) ostream)
  "Serializes a message object of type '<WeatherData>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'baro_millibars))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temp_C))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wind_mph))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wind_direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WeatherData>) istream)
  "Deserializes a message object of type '<WeatherData>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'baro_millibars) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temp_C) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wind_mph) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wind_direction) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WeatherData>)))
  "Returns string type for a message object of type '<WeatherData>"
  "autopilot_bridge/WeatherData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WeatherData)))
  "Returns string type for a message object of type 'WeatherData"
  "autopilot_bridge/WeatherData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WeatherData>)))
  "Returns md5sum for a message object of type '<WeatherData>"
  "ad6ecb730008621b85e84342a94f123f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WeatherData)))
  "Returns md5sum for a message object of type 'WeatherData"
  "ad6ecb730008621b85e84342a94f123f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WeatherData>)))
  "Returns full string definition for message of type '<WeatherData>"
  (cl:format cl:nil "float32 baro_millibars~%float32 temp_C~%float32 wind_mph~%float32 wind_direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WeatherData)))
  "Returns full string definition for message of type 'WeatherData"
  (cl:format cl:nil "float32 baro_millibars~%float32 temp_C~%float32 wind_mph~%float32 wind_direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WeatherData>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WeatherData>))
  "Converts a ROS message object to a list"
  (cl:list 'WeatherData
    (cl:cons ':baro_millibars (baro_millibars msg))
    (cl:cons ':temp_C (temp_C msg))
    (cl:cons ':wind_mph (wind_mph msg))
    (cl:cons ':wind_direction (wind_direction msg))
))
