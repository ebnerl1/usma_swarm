; Auto-generated. Do not edit!


(cl:in-package ap_msgs-msg)


;//! \htmlinclude BehaviorParameters.msg.html

(cl:defclass <BehaviorParameters> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (params
    :reader params
    :initarg :params
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass BehaviorParameters (<BehaviorParameters>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BehaviorParameters>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BehaviorParameters)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ap_msgs-msg:<BehaviorParameters> is deprecated: use ap_msgs-msg:BehaviorParameters instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <BehaviorParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:id-val is deprecated.  Use ap_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'params-val :lambda-list '(m))
(cl:defmethod params-val ((m <BehaviorParameters>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ap_msgs-msg:params-val is deprecated.  Use ap_msgs-msg:params instead.")
  (params m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BehaviorParameters>) ostream)
  "Serializes a message object of type '<BehaviorParameters>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'params))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'params))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BehaviorParameters>) istream)
  "Deserializes a message object of type '<BehaviorParameters>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'params) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'params)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BehaviorParameters>)))
  "Returns string type for a message object of type '<BehaviorParameters>"
  "ap_msgs/BehaviorParameters")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BehaviorParameters)))
  "Returns string type for a message object of type 'BehaviorParameters"
  "ap_msgs/BehaviorParameters")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BehaviorParameters>)))
  "Returns md5sum for a message object of type '<BehaviorParameters>"
  "af1234d6bfae4749ec1409770312d0f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BehaviorParameters)))
  "Returns md5sum for a message object of type 'BehaviorParameters"
  "af1234d6bfae4749ec1409770312d0f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BehaviorParameters>)))
  "Returns full string definition for message of type '<BehaviorParameters>"
  (cl:format cl:nil "uint8 id~%uint8[] params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BehaviorParameters)))
  "Returns full string definition for message of type 'BehaviorParameters"
  (cl:format cl:nil "uint8 id~%uint8[] params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BehaviorParameters>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'params) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BehaviorParameters>))
  "Converts a ROS message object to a list"
  (cl:list 'BehaviorParameters
    (cl:cons ':id (id msg))
    (cl:cons ':params (params msg))
))
