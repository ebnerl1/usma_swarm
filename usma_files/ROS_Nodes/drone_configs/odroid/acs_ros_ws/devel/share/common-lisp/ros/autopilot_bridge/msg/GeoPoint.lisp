; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-msg)


;//! \htmlinclude GeoPoint.msg.html

(cl:defclass <GeoPoint> (roslisp-msg-protocol:ros-message)
  ((lat
    :reader lat
    :initarg :lat
    :type cl:float
    :initform 0.0)
   (lon
    :reader lon
    :initarg :lon
    :type cl:float
    :initform 0.0)
   (alt
    :reader alt
    :initarg :alt
    :type cl:float
    :initform 0.0)
   (rel_alt
    :reader rel_alt
    :initarg :rel_alt
    :type cl:float
    :initform 0.0)
   (using_alt
    :reader using_alt
    :initarg :using_alt
    :type cl:boolean
    :initform cl:nil)
   (using_rel_alt
    :reader using_rel_alt
    :initarg :using_rel_alt
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GeoPoint (<GeoPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GeoPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GeoPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-msg:<GeoPoint> is deprecated: use autopilot_bridge-msg:GeoPoint instead.")))

(cl:ensure-generic-function 'lat-val :lambda-list '(m))
(cl:defmethod lat-val ((m <GeoPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:lat-val is deprecated.  Use autopilot_bridge-msg:lat instead.")
  (lat m))

(cl:ensure-generic-function 'lon-val :lambda-list '(m))
(cl:defmethod lon-val ((m <GeoPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:lon-val is deprecated.  Use autopilot_bridge-msg:lon instead.")
  (lon m))

(cl:ensure-generic-function 'alt-val :lambda-list '(m))
(cl:defmethod alt-val ((m <GeoPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:alt-val is deprecated.  Use autopilot_bridge-msg:alt instead.")
  (alt m))

(cl:ensure-generic-function 'rel_alt-val :lambda-list '(m))
(cl:defmethod rel_alt-val ((m <GeoPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:rel_alt-val is deprecated.  Use autopilot_bridge-msg:rel_alt instead.")
  (rel_alt m))

(cl:ensure-generic-function 'using_alt-val :lambda-list '(m))
(cl:defmethod using_alt-val ((m <GeoPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:using_alt-val is deprecated.  Use autopilot_bridge-msg:using_alt instead.")
  (using_alt m))

(cl:ensure-generic-function 'using_rel_alt-val :lambda-list '(m))
(cl:defmethod using_rel_alt-val ((m <GeoPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:using_rel_alt-val is deprecated.  Use autopilot_bridge-msg:using_rel_alt instead.")
  (using_rel_alt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GeoPoint>) ostream)
  "Serializes a message object of type '<GeoPoint>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'alt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rel_alt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'using_alt) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'using_rel_alt) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GeoPoint>) istream)
  "Deserializes a message object of type '<GeoPoint>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lat) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lon) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'alt) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rel_alt) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'using_alt) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'using_rel_alt) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GeoPoint>)))
  "Returns string type for a message object of type '<GeoPoint>"
  "autopilot_bridge/GeoPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GeoPoint)))
  "Returns string type for a message object of type 'GeoPoint"
  "autopilot_bridge/GeoPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GeoPoint>)))
  "Returns md5sum for a message object of type '<GeoPoint>"
  "c121d48ffba7006a7ab486f309ebbbb8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GeoPoint)))
  "Returns md5sum for a message object of type 'GeoPoint"
  "c121d48ffba7006a7ab486f309ebbbb8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GeoPoint>)))
  "Returns full string definition for message of type '<GeoPoint>"
  (cl:format cl:nil "# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GeoPoint)))
  "Returns full string definition for message of type 'GeoPoint"
  (cl:format cl:nil "# WGS84 latitude, longitude, and MSL altitude~%float64 lat~%float64 lon~%float64 alt~%~%# Relative altitude (custom definable, but generally set w.r.t. some home point)~%float64 rel_alt~%~%# Flag field for which altitudes are valid~%bool using_alt~%bool using_rel_alt~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GeoPoint>))
  (cl:+ 0
     8
     8
     8
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GeoPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'GeoPoint
    (cl:cons ':lat (lat msg))
    (cl:cons ':lon (lon msg))
    (cl:cons ':alt (alt msg))
    (cl:cons ':rel_alt (rel_alt msg))
    (cl:cons ':using_alt (using_alt msg))
    (cl:cons ':using_rel_alt (using_rel_alt msg))
))
