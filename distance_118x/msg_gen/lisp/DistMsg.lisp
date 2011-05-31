; Auto-generated. Do not edit!


(cl:in-package distance_118x-msg)


;//! \htmlinclude DistMsg.msg.html

(cl:defclass <DistMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass DistMsg (<DistMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name distance_118x-msg:<DistMsg> is deprecated: use distance_118x-msg:DistMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DistMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distance_118x-msg:header-val is deprecated.  Use distance_118x-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <DistMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distance_118x-msg:distance-val is deprecated.  Use distance_118x-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistMsg>) ostream)
  "Serializes a message object of type '<DistMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistMsg>) istream)
  "Deserializes a message object of type '<DistMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistMsg>)))
  "Returns string type for a message object of type '<DistMsg>"
  "distance_118x/DistMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistMsg)))
  "Returns string type for a message object of type 'DistMsg"
  "distance_118x/DistMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistMsg>)))
  "Returns md5sum for a message object of type '<DistMsg>"
  "4c739b0ad83c2914df6d41fe1b87d25f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistMsg)))
  "Returns md5sum for a message object of type 'DistMsg"
  "4c739b0ad83c2914df6d41fe1b87d25f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistMsg>)))
  "Returns full string definition for message of type '<DistMsg>"
  (cl:format cl:nil "Header header~%float32 distance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistMsg)))
  "Returns full string definition for message of type 'DistMsg"
  (cl:format cl:nil "Header header~%float32 distance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'DistMsg
    (cl:cons ':header (header msg))
    (cl:cons ':distance (distance msg))
))
