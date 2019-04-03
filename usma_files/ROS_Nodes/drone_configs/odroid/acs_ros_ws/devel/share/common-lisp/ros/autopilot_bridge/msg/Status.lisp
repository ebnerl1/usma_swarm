; Auto-generated. Do not edit!


(cl:in-package autopilot_bridge-msg)


;//! \htmlinclude Status.msg.html

(cl:defclass <Status> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (armed
    :reader armed
    :initarg :armed
    :type cl:boolean
    :initform cl:nil)
   (ahrs_ok
    :reader ahrs_ok
    :initarg :ahrs_ok
    :type cl:boolean
    :initform cl:nil)
   (alt_rel
    :reader alt_rel
    :initarg :alt_rel
    :type cl:integer
    :initform 0)
   (as_ok
    :reader as_ok
    :initarg :as_ok
    :type cl:boolean
    :initform cl:nil)
   (as_read
    :reader as_read
    :initarg :as_read
    :type cl:float
    :initform 0.0)
   (fence_status
    :reader fence_status
    :initarg :fence_status
    :type cl:fixnum
    :initform 0)
   (gps_ok
    :reader gps_ok
    :initarg :gps_ok
    :type cl:boolean
    :initform cl:nil)
   (gps_sats
    :reader gps_sats
    :initarg :gps_sats
    :type cl:fixnum
    :initform 0)
   (gps_eph
    :reader gps_eph
    :initarg :gps_eph
    :type cl:fixnum
    :initform 0)
   (ins_ok
    :reader ins_ok
    :initarg :ins_ok
    :type cl:boolean
    :initform cl:nil)
   (mag_ok
    :reader mag_ok
    :initarg :mag_ok
    :type cl:boolean
    :initform cl:nil)
   (mis_cur
    :reader mis_cur
    :initarg :mis_cur
    :type cl:fixnum
    :initform 0)
   (pwr_ok
    :reader pwr_ok
    :initarg :pwr_ok
    :type cl:boolean
    :initform cl:nil)
   (pwr_batt_rem
    :reader pwr_batt_rem
    :initarg :pwr_batt_rem
    :type cl:fixnum
    :initform 0)
   (pwr_batt_vcc
    :reader pwr_batt_vcc
    :initarg :pwr_batt_vcc
    :type cl:fixnum
    :initform 0)
   (pwr_batt_cur
    :reader pwr_batt_cur
    :initarg :pwr_batt_cur
    :type cl:fixnum
    :initform 0)
   (vehicle_type
    :reader vehicle_type
    :initarg :vehicle_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Status (<Status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name autopilot_bridge-msg:<Status> is deprecated: use autopilot_bridge-msg:Status instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:header-val is deprecated.  Use autopilot_bridge-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:mode-val is deprecated.  Use autopilot_bridge-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'armed-val :lambda-list '(m))
(cl:defmethod armed-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:armed-val is deprecated.  Use autopilot_bridge-msg:armed instead.")
  (armed m))

(cl:ensure-generic-function 'ahrs_ok-val :lambda-list '(m))
(cl:defmethod ahrs_ok-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:ahrs_ok-val is deprecated.  Use autopilot_bridge-msg:ahrs_ok instead.")
  (ahrs_ok m))

(cl:ensure-generic-function 'alt_rel-val :lambda-list '(m))
(cl:defmethod alt_rel-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:alt_rel-val is deprecated.  Use autopilot_bridge-msg:alt_rel instead.")
  (alt_rel m))

(cl:ensure-generic-function 'as_ok-val :lambda-list '(m))
(cl:defmethod as_ok-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:as_ok-val is deprecated.  Use autopilot_bridge-msg:as_ok instead.")
  (as_ok m))

(cl:ensure-generic-function 'as_read-val :lambda-list '(m))
(cl:defmethod as_read-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:as_read-val is deprecated.  Use autopilot_bridge-msg:as_read instead.")
  (as_read m))

(cl:ensure-generic-function 'fence_status-val :lambda-list '(m))
(cl:defmethod fence_status-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:fence_status-val is deprecated.  Use autopilot_bridge-msg:fence_status instead.")
  (fence_status m))

(cl:ensure-generic-function 'gps_ok-val :lambda-list '(m))
(cl:defmethod gps_ok-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:gps_ok-val is deprecated.  Use autopilot_bridge-msg:gps_ok instead.")
  (gps_ok m))

(cl:ensure-generic-function 'gps_sats-val :lambda-list '(m))
(cl:defmethod gps_sats-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:gps_sats-val is deprecated.  Use autopilot_bridge-msg:gps_sats instead.")
  (gps_sats m))

(cl:ensure-generic-function 'gps_eph-val :lambda-list '(m))
(cl:defmethod gps_eph-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:gps_eph-val is deprecated.  Use autopilot_bridge-msg:gps_eph instead.")
  (gps_eph m))

(cl:ensure-generic-function 'ins_ok-val :lambda-list '(m))
(cl:defmethod ins_ok-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:ins_ok-val is deprecated.  Use autopilot_bridge-msg:ins_ok instead.")
  (ins_ok m))

(cl:ensure-generic-function 'mag_ok-val :lambda-list '(m))
(cl:defmethod mag_ok-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:mag_ok-val is deprecated.  Use autopilot_bridge-msg:mag_ok instead.")
  (mag_ok m))

(cl:ensure-generic-function 'mis_cur-val :lambda-list '(m))
(cl:defmethod mis_cur-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:mis_cur-val is deprecated.  Use autopilot_bridge-msg:mis_cur instead.")
  (mis_cur m))

(cl:ensure-generic-function 'pwr_ok-val :lambda-list '(m))
(cl:defmethod pwr_ok-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:pwr_ok-val is deprecated.  Use autopilot_bridge-msg:pwr_ok instead.")
  (pwr_ok m))

(cl:ensure-generic-function 'pwr_batt_rem-val :lambda-list '(m))
(cl:defmethod pwr_batt_rem-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:pwr_batt_rem-val is deprecated.  Use autopilot_bridge-msg:pwr_batt_rem instead.")
  (pwr_batt_rem m))

(cl:ensure-generic-function 'pwr_batt_vcc-val :lambda-list '(m))
(cl:defmethod pwr_batt_vcc-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:pwr_batt_vcc-val is deprecated.  Use autopilot_bridge-msg:pwr_batt_vcc instead.")
  (pwr_batt_vcc m))

(cl:ensure-generic-function 'pwr_batt_cur-val :lambda-list '(m))
(cl:defmethod pwr_batt_cur-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:pwr_batt_cur-val is deprecated.  Use autopilot_bridge-msg:pwr_batt_cur instead.")
  (pwr_batt_cur m))

(cl:ensure-generic-function 'vehicle_type-val :lambda-list '(m))
(cl:defmethod vehicle_type-val ((m <Status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader autopilot_bridge-msg:vehicle_type-val is deprecated.  Use autopilot_bridge-msg:vehicle_type instead.")
  (vehicle_type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Status>)))
    "Constants for message type '<Status>"
  '((:MODE_RALLY . 0)
    (:MODE_MANUAL . 1)
    (:MODE_FBWA . 2)
    (:MODE_GUIDED . 3)
    (:MODE_AUTO . 4)
    (:MODE_FBWB . 5)
    (:MODE_CIRCLE . 6)
    (:MODE_LOITER . 7)
    (:MODE_INITIALIZING . 8)
    (:MODE_STABILIZE . 9)
    (:MODE_ALT_HOLD . 10)
    (:MODE_LAND . 11)
    (:MODE_UNKNOWN . 15)
    (:FENCE_NOT_BREACHED . 0)
    (:FENCE_BREACHED . 1)
    (:FENCE_DISABLED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Status)))
    "Constants for message type 'Status"
  '((:MODE_RALLY . 0)
    (:MODE_MANUAL . 1)
    (:MODE_FBWA . 2)
    (:MODE_GUIDED . 3)
    (:MODE_AUTO . 4)
    (:MODE_FBWB . 5)
    (:MODE_CIRCLE . 6)
    (:MODE_LOITER . 7)
    (:MODE_INITIALIZING . 8)
    (:MODE_STABILIZE . 9)
    (:MODE_ALT_HOLD . 10)
    (:MODE_LAND . 11)
    (:MODE_UNKNOWN . 15)
    (:FENCE_NOT_BREACHED . 0)
    (:FENCE_BREACHED . 1)
    (:FENCE_DISABLED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Status>) ostream)
  "Serializes a message object of type '<Status>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'armed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ahrs_ok) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'alt_rel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'as_ok) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'as_read))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fence_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gps_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_sats)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_eph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gps_eph)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ins_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mag_ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mis_cur)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mis_cur)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pwr_ok) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'pwr_batt_rem)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pwr_batt_vcc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pwr_batt_cur)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Status>) istream)
  "Deserializes a message object of type '<Status>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'armed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ahrs_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'alt_rel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'as_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'as_read) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fence_status)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gps_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_sats)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gps_eph)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gps_eph)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ins_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mag_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mis_cur)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mis_cur)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pwr_ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwr_batt_rem) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwr_batt_vcc) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pwr_batt_cur) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vehicle_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Status>)))
  "Returns string type for a message object of type '<Status>"
  "autopilot_bridge/Status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Status)))
  "Returns string type for a message object of type 'Status"
  "autopilot_bridge/Status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Status>)))
  "Returns md5sum for a message object of type '<Status>"
  "55a39af1aa38a378c9b9ffef2b7cbad9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Status)))
  "Returns md5sum for a message object of type 'Status"
  "55a39af1aa38a378c9b9ffef2b7cbad9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Status>)))
  "Returns full string definition for message of type '<Status>"
  (cl:format cl:nil "uint8 MODE_RALLY=0~%uint8 MODE_MANUAL=1~%uint8 MODE_FBWA=2~%uint8 MODE_GUIDED=3~%uint8 MODE_AUTO=4~%uint8 MODE_FBWB=5~%uint8 MODE_CIRCLE=6~%uint8 MODE_LOITER=7~%uint8 MODE_INITIALIZING=8~%uint8 MODE_STABILIZE=9~%uint8 MODE_ALT_HOLD=10~%uint8 MODE_LAND=11~%uint8 MODE_UNKNOWN=15~%~%uint8 FENCE_NOT_BREACHED=0~%uint8 FENCE_BREACHED=1~%uint8 FENCE_DISABLED=2~%~%Header  header~%uint8   mode~%bool    armed~%bool    ahrs_ok~%int32   alt_rel~%bool    as_ok~%float32 as_read~%uint8   fence_status~%bool    gps_ok~%uint8   gps_sats~%uint16  gps_eph~%bool    ins_ok~%bool    mag_ok~%uint16  mis_cur~%bool    pwr_ok~%int16   pwr_batt_rem~%int16   pwr_batt_vcc~%int16   pwr_batt_cur~%uint8   vehicle_type~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Status)))
  "Returns full string definition for message of type 'Status"
  (cl:format cl:nil "uint8 MODE_RALLY=0~%uint8 MODE_MANUAL=1~%uint8 MODE_FBWA=2~%uint8 MODE_GUIDED=3~%uint8 MODE_AUTO=4~%uint8 MODE_FBWB=5~%uint8 MODE_CIRCLE=6~%uint8 MODE_LOITER=7~%uint8 MODE_INITIALIZING=8~%uint8 MODE_STABILIZE=9~%uint8 MODE_ALT_HOLD=10~%uint8 MODE_LAND=11~%uint8 MODE_UNKNOWN=15~%~%uint8 FENCE_NOT_BREACHED=0~%uint8 FENCE_BREACHED=1~%uint8 FENCE_DISABLED=2~%~%Header  header~%uint8   mode~%bool    armed~%bool    ahrs_ok~%int32   alt_rel~%bool    as_ok~%float32 as_read~%uint8   fence_status~%bool    gps_ok~%uint8   gps_sats~%uint16  gps_eph~%bool    ins_ok~%bool    mag_ok~%uint16  mis_cur~%bool    pwr_ok~%int16   pwr_batt_rem~%int16   pwr_batt_vcc~%int16   pwr_batt_cur~%uint8   vehicle_type~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Status>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4
     1
     4
     1
     1
     1
     2
     1
     1
     2
     1
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Status>))
  "Converts a ROS message object to a list"
  (cl:list 'Status
    (cl:cons ':header (header msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':armed (armed msg))
    (cl:cons ':ahrs_ok (ahrs_ok msg))
    (cl:cons ':alt_rel (alt_rel msg))
    (cl:cons ':as_ok (as_ok msg))
    (cl:cons ':as_read (as_read msg))
    (cl:cons ':fence_status (fence_status msg))
    (cl:cons ':gps_ok (gps_ok msg))
    (cl:cons ':gps_sats (gps_sats msg))
    (cl:cons ':gps_eph (gps_eph msg))
    (cl:cons ':ins_ok (ins_ok msg))
    (cl:cons ':mag_ok (mag_ok msg))
    (cl:cons ':mis_cur (mis_cur msg))
    (cl:cons ':pwr_ok (pwr_ok msg))
    (cl:cons ':pwr_batt_rem (pwr_batt_rem msg))
    (cl:cons ':pwr_batt_vcc (pwr_batt_vcc msg))
    (cl:cons ':pwr_batt_cur (pwr_batt_cur msg))
    (cl:cons ':vehicle_type (vehicle_type msg))
))
