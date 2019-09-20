; Auto-generated. Do not edit!


(cl:in-package way_point_wamv-srv)


;//! \htmlinclude way_point_cmd-request.msg.html

(cl:defclass <way_point_cmd-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass way_point_cmd-request (<way_point_cmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <way_point_cmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'way_point_cmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name way_point_wamv-srv:<way_point_cmd-request> is deprecated: use way_point_wamv-srv:way_point_cmd-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <way_point_cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader way_point_wamv-srv:command-val is deprecated.  Use way_point_wamv-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <way_point_cmd-request>) ostream)
  "Serializes a message object of type '<way_point_cmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <way_point_cmd-request>) istream)
  "Deserializes a message object of type '<way_point_cmd-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<way_point_cmd-request>)))
  "Returns string type for a service object of type '<way_point_cmd-request>"
  "way_point_wamv/way_point_cmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'way_point_cmd-request)))
  "Returns string type for a service object of type 'way_point_cmd-request"
  "way_point_wamv/way_point_cmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<way_point_cmd-request>)))
  "Returns md5sum for a message object of type '<way_point_cmd-request>"
  "9115d0e027e87d5bbec3f644195b57c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'way_point_cmd-request)))
  "Returns md5sum for a message object of type 'way_point_cmd-request"
  "9115d0e027e87d5bbec3f644195b57c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<way_point_cmd-request>)))
  "Returns full string definition for message of type '<way_point_cmd-request>"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'way_point_cmd-request)))
  "Returns full string definition for message of type 'way_point_cmd-request"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <way_point_cmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <way_point_cmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'way_point_cmd-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude way_point_cmd-response.msg.html

(cl:defclass <way_point_cmd-response> (roslisp-msg-protocol:ros-message)
  ((recieved
    :reader recieved
    :initarg :recieved
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass way_point_cmd-response (<way_point_cmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <way_point_cmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'way_point_cmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name way_point_wamv-srv:<way_point_cmd-response> is deprecated: use way_point_wamv-srv:way_point_cmd-response instead.")))

(cl:ensure-generic-function 'recieved-val :lambda-list '(m))
(cl:defmethod recieved-val ((m <way_point_cmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader way_point_wamv-srv:recieved-val is deprecated.  Use way_point_wamv-srv:recieved instead.")
  (recieved m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <way_point_cmd-response>) ostream)
  "Serializes a message object of type '<way_point_cmd-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'recieved) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <way_point_cmd-response>) istream)
  "Deserializes a message object of type '<way_point_cmd-response>"
    (cl:setf (cl:slot-value msg 'recieved) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<way_point_cmd-response>)))
  "Returns string type for a service object of type '<way_point_cmd-response>"
  "way_point_wamv/way_point_cmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'way_point_cmd-response)))
  "Returns string type for a service object of type 'way_point_cmd-response"
  "way_point_wamv/way_point_cmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<way_point_cmd-response>)))
  "Returns md5sum for a message object of type '<way_point_cmd-response>"
  "9115d0e027e87d5bbec3f644195b57c4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'way_point_cmd-response)))
  "Returns md5sum for a message object of type 'way_point_cmd-response"
  "9115d0e027e87d5bbec3f644195b57c4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<way_point_cmd-response>)))
  "Returns full string definition for message of type '<way_point_cmd-response>"
  (cl:format cl:nil "bool recieved~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'way_point_cmd-response)))
  "Returns full string definition for message of type 'way_point_cmd-response"
  (cl:format cl:nil "bool recieved~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <way_point_cmd-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <way_point_cmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'way_point_cmd-response
    (cl:cons ':recieved (recieved msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'way_point_cmd)))
  'way_point_cmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'way_point_cmd)))
  'way_point_cmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'way_point_cmd)))
  "Returns string type for a service object of type '<way_point_cmd>"
  "way_point_wamv/way_point_cmd")