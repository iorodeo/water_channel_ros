; Auto-generated. Do not edit!


(cl:in-package controller-srv)


;//! \htmlinclude ControllerCmd-request.msg.html

(cl:defclass <ControllerCmd-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ControllerCmd-request (<ControllerCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<ControllerCmd-request> is deprecated: use controller-srv:ControllerCmd-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <ControllerCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:cmd-val is deprecated.  Use controller-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'valueString-val :lambda-list '(m))
(cl:defmethod valueString-val ((m <ControllerCmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:valueString-val is deprecated.  Use controller-srv:valueString instead.")
  (valueString m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerCmd-request>) ostream)
  "Serializes a message object of type '<ControllerCmd-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerCmd-request>) istream)
  "Deserializes a message object of type '<ControllerCmd-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerCmd-request>)))
  "Returns string type for a service object of type '<ControllerCmd-request>"
  "controller/ControllerCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerCmd-request)))
  "Returns string type for a service object of type 'ControllerCmd-request"
  "controller/ControllerCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerCmd-request>)))
  "Returns md5sum for a message object of type '<ControllerCmd-request>"
  "4673beee203938f168b8771b628c2851")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerCmd-request)))
  "Returns md5sum for a message object of type 'ControllerCmd-request"
  "4673beee203938f168b8771b628c2851")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerCmd-request>)))
  "Returns full string definition for message of type '<ControllerCmd-request>"
  (cl:format cl:nil "string cmd~%string valueString~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerCmd-request)))
  "Returns full string definition for message of type 'ControllerCmd-request"
  (cl:format cl:nil "string cmd~%string valueString~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerCmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cmd))
     4 (cl:length (cl:slot-value msg 'valueString))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerCmd-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':valueString (valueString msg))
))
;//! \htmlinclude ControllerCmd-response.msg.html

(cl:defclass <ControllerCmd-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControllerCmd-response (<ControllerCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<ControllerCmd-response> is deprecated: use controller-srv:ControllerCmd-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerCmd-response>) ostream)
  "Serializes a message object of type '<ControllerCmd-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerCmd-response>) istream)
  "Deserializes a message object of type '<ControllerCmd-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerCmd-response>)))
  "Returns string type for a service object of type '<ControllerCmd-response>"
  "controller/ControllerCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerCmd-response)))
  "Returns string type for a service object of type 'ControllerCmd-response"
  "controller/ControllerCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerCmd-response>)))
  "Returns md5sum for a message object of type '<ControllerCmd-response>"
  "4673beee203938f168b8771b628c2851")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerCmd-response)))
  "Returns md5sum for a message object of type 'ControllerCmd-response"
  "4673beee203938f168b8771b628c2851")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerCmd-response>)))
  "Returns full string definition for message of type '<ControllerCmd-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerCmd-response)))
  "Returns full string definition for message of type 'ControllerCmd-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerCmd-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerCmd-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControllerCmd)))
  'ControllerCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControllerCmd)))
  'ControllerCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerCmd)))
  "Returns string type for a service object of type '<ControllerCmd>"
  "controller/ControllerCmd")
