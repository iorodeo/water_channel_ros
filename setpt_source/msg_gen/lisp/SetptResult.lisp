; Auto-generated. Do not edit!


(cl:in-package setpt_source-msg)


;//! \htmlinclude SetptResult.msg.html

(cl:defclass <SetptResult> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetptResult (<SetptResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetptResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetptResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name setpt_source-msg:<SetptResult> is deprecated: use setpt_source-msg:SetptResult instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <SetptResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader setpt_source-msg:position-val is deprecated.  Use setpt_source-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetptResult>) ostream)
  "Serializes a message object of type '<SetptResult>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetptResult>) istream)
  "Deserializes a message object of type '<SetptResult>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetptResult>)))
  "Returns string type for a message object of type '<SetptResult>"
  "setpt_source/SetptResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetptResult)))
  "Returns string type for a message object of type 'SetptResult"
  "setpt_source/SetptResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetptResult>)))
  "Returns md5sum for a message object of type '<SetptResult>"
  "e4e11b3af29ed247b2bd150d3f4540f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetptResult)))
  "Returns md5sum for a message object of type 'SetptResult"
  "e4e11b3af29ed247b2bd150d3f4540f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetptResult>)))
  "Returns full string definition for message of type '<SetptResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%float32 position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetptResult)))
  "Returns full string definition for message of type 'SetptResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%float32 position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetptResult>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetptResult>))
  "Converts a ROS message object to a list"
  (cl:list 'SetptResult
    (cl:cons ':position (position msg))
))
