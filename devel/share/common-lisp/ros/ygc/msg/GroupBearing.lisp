; Auto-generated. Do not edit!


(cl:in-package ygc-msg)


;//! \htmlinclude GroupBearing.msg.html

(cl:defclass <GroupBearing> (roslisp-msg-protocol:ros-message)
  ((bearings
    :reader bearings
    :initarg :bearings
    :type (cl:vector ygc-msg:Bearing2D)
   :initform (cl:make-array 0 :element-type 'ygc-msg:Bearing2D :initial-element (cl:make-instance 'ygc-msg:Bearing2D))))
)

(cl:defclass GroupBearing (<GroupBearing>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GroupBearing>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GroupBearing)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ygc-msg:<GroupBearing> is deprecated: use ygc-msg:GroupBearing instead.")))

(cl:ensure-generic-function 'bearings-val :lambda-list '(m))
(cl:defmethod bearings-val ((m <GroupBearing>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ygc-msg:bearings-val is deprecated.  Use ygc-msg:bearings instead.")
  (bearings m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GroupBearing>) ostream)
  "Serializes a message object of type '<GroupBearing>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bearings))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bearings))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GroupBearing>) istream)
  "Deserializes a message object of type '<GroupBearing>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bearings) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bearings)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ygc-msg:Bearing2D))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GroupBearing>)))
  "Returns string type for a message object of type '<GroupBearing>"
  "ygc/GroupBearing")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GroupBearing)))
  "Returns string type for a message object of type 'GroupBearing"
  "ygc/GroupBearing")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GroupBearing>)))
  "Returns md5sum for a message object of type '<GroupBearing>"
  "f03b323f78db40d62b3828c934a95919")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GroupBearing)))
  "Returns md5sum for a message object of type 'GroupBearing"
  "f03b323f78db40d62b3828c934a95919")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GroupBearing>)))
  "Returns full string definition for message of type '<GroupBearing>"
  (cl:format cl:nil "Bearing2D[] bearings~%~%================================================================================~%MSG: ygc/Bearing2D~%float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GroupBearing)))
  "Returns full string definition for message of type 'GroupBearing"
  (cl:format cl:nil "Bearing2D[] bearings~%~%================================================================================~%MSG: ygc/Bearing2D~%float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GroupBearing>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bearings) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GroupBearing>))
  "Converts a ROS message object to a list"
  (cl:list 'GroupBearing
    (cl:cons ':bearings (bearings msg))
))
