; Auto-generated. Do not edit!


(cl:in-package distance_118x-srv)


;//! \htmlinclude DistSensorCtl-request.msg.html

(cl:defclass <DistSensorCtl-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:string
    :initform "")
   (valueString
    :reader valueString
    :initarg :valueString
    :type cl:string
    :initform ""))
)

(cl:defclass DistSensorCtl-request (<DistSensorCtl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistSensorCtl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistSensorCtl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name distance_118x-srv:<DistSensorCtl-request> is deprecated: use distance_118x-srv:DistSensorCtl-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <DistSensorCtl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distance_118x-srv:cmd-val is deprecated.  Use distance_118x-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'valueString-val :lambda-list '(m))
(cl:defmethod valueString-val ((m <DistSensorCtl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader distance_118x-srv:valueString-val is deprecated.  Use distance_118x-srv:valueString instead.")
  (valueString m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistSensorCtl-request>) ostream)
  "Serializes a message object of type '<DistSensorCtl-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'valueString))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'valueString))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistSensorCtl-request>) istream)
  "Deserializes a message object of type '<DistSensorCtl-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'valueString) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'valueString) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistSensorCtl-request>)))
  "Returns string type for a service object of type '<DistSensorCtl-request>"
  "distance_118x/DistSensorCtlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistSensorCtl-request)))
  "Returns string type for a service object of type 'DistSensorCtl-request"
  "distance_118x/DistSensorCtlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistSensorCtl-request>)))
  "Returns md5sum for a message object of type '<DistSensorCtl-request>"
  "4673beee203938f168b8771b628c2851")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistSensorCtl-request)))
  "Returns md5sum for a message object of type 'DistSensorCtl-request"
  "4673beee203938f168b8771b628c2851")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistSensorCtl-request>)))
  "Returns full string definition for message of type '<DistSensorCtl-request>"
  (cl:format cl:nil "string cmd~%string valueString~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistSensorCtl-request)))
  "Returns full string definition for message of type 'DistSensorCtl-request"
  (cl:format cl:nil "string cmd~%string valueString~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistSensorCtl-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cmd))
     4 (cl:length (cl:slot-value msg 'valueString))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistSensorCtl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DistSensorCtl-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':valueString (valueString msg))
))
;//! \htmlinclude DistSensorCtl-response.msg.html

(cl:defclass <DistSensorCtl-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass DistSensorCtl-response (<DistSensorCtl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistSensorCtl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistSensorCtl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name distance_118x-srv:<DistSensorCtl-response> is deprecated: use distance_118x-srv:DistSensorCtl-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistSensorCtl-response>) ostream)
  "Serializes a message object of type '<DistSensorCtl-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistSensorCtl-response>) istream)
  "Deserializes a message object of type '<DistSensorCtl-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistSensorCtl-response>)))
  "Returns string type for a service object of type '<DistSensorCtl-response>"
  "distance_118x/DistSensorCtlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistSensorCtl-response)))
  "Returns string type for a service object of type 'DistSensorCtl-response"
  "distance_118x/DistSensorCtlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistSensorCtl-response>)))
  "Returns md5sum for a message object of type '<DistSensorCtl-response>"
  "4673beee203938f168b8771b628c2851")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistSensorCtl-response)))
  "Returns md5sum for a message object of type 'DistSensorCtl-response"
  "4673beee203938f168b8771b628c2851")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistSensorCtl-response>)))
  "Returns full string definition for message of type '<DistSensorCtl-response>"
  (cl:format cl:nil "~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistSensorCtl-response)))
  "Returns full string definition for message of type 'DistSensorCtl-response"
  (cl:format cl:nil "~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistSensorCtl-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistSensorCtl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DistSensorCtl-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DistSensorCtl)))
  'DistSensorCtl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DistSensorCtl)))
  'DistSensorCtl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistSensorCtl)))
  "Returns string type for a service object of type '<DistSensorCtl>"
  "distance_118x/DistSensorCtl")
