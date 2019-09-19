; Auto-generated. Do not edit!


(cl:in-package way_point_wamv-srv)


;//! \htmlinclude add_way_point-request.msg.html

(cl:defclass <add_way_point-request> (roslisp-msg-protocol:ros-message)
  ((latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (minutes
    :reader minutes
    :initarg :minutes
    :type cl:float
    :initform 0.0))
)

(cl:defclass add_way_point-request (<add_way_point-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <add_way_point-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'add_way_point-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name way_point_wamv-srv:<add_way_point-request> is deprecated: use way_point_wamv-srv:add_way_point-request instead.")))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <add_way_point-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader way_point_wamv-srv:latitude-val is deprecated.  Use way_point_wamv-srv:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <add_way_point-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader way_point_wamv-srv:longitude-val is deprecated.  Use way_point_wamv-srv:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'minutes-val :lambda-list '(m))
(cl:defmethod minutes-val ((m <add_way_point-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader way_point_wamv-srv:minutes-val is deprecated.  Use way_point_wamv-srv:minutes instead.")
  (minutes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <add_way_point-request>) ostream)
  "Serializes a message object of type '<add_way_point-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'minutes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <add_way_point-request>) istream)
  "Deserializes a message object of type '<add_way_point-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minutes) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<add_way_point-request>)))
  "Returns string type for a service object of type '<add_way_point-request>"
  "way_point_wamv/add_way_pointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'add_way_point-request)))
  "Returns string type for a service object of type 'add_way_point-request"
  "way_point_wamv/add_way_pointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<add_way_point-request>)))
  "Returns md5sum for a message object of type '<add_way_point-request>"
  "8f7e4b30056b33ab373baed5e3088955")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'add_way_point-request)))
  "Returns md5sum for a message object of type 'add_way_point-request"
  "8f7e4b30056b33ab373baed5e3088955")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<add_way_point-request>)))
  "Returns full string definition for message of type '<add_way_point-request>"
  (cl:format cl:nil "float64 latitude~%float64 longitude~%float64 minutes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'add_way_point-request)))
  "Returns full string definition for message of type 'add_way_point-request"
  (cl:format cl:nil "float64 latitude~%float64 longitude~%float64 minutes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <add_way_point-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <add_way_point-request>))
  "Converts a ROS message object to a list"
  (cl:list 'add_way_point-request
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':minutes (minutes msg))
))
;//! \htmlinclude add_way_point-response.msg.html

(cl:defclass <add_way_point-response> (roslisp-msg-protocol:ros-message)
  ((recieved
    :reader recieved
    :initarg :recieved
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass add_way_point-response (<add_way_point-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <add_way_point-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'add_way_point-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name way_point_wamv-srv:<add_way_point-response> is deprecated: use way_point_wamv-srv:add_way_point-response instead.")))

(cl:ensure-generic-function 'recieved-val :lambda-list '(m))
(cl:defmethod recieved-val ((m <add_way_point-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader way_point_wamv-srv:recieved-val is deprecated.  Use way_point_wamv-srv:recieved instead.")
  (recieved m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <add_way_point-response>) ostream)
  "Serializes a message object of type '<add_way_point-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'recieved) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <add_way_point-response>) istream)
  "Deserializes a message object of type '<add_way_point-response>"
    (cl:setf (cl:slot-value msg 'recieved) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<add_way_point-response>)))
  "Returns string type for a service object of type '<add_way_point-response>"
  "way_point_wamv/add_way_pointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'add_way_point-response)))
  "Returns string type for a service object of type 'add_way_point-response"
  "way_point_wamv/add_way_pointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<add_way_point-response>)))
  "Returns md5sum for a message object of type '<add_way_point-response>"
  "8f7e4b30056b33ab373baed5e3088955")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'add_way_point-response)))
  "Returns md5sum for a message object of type 'add_way_point-response"
  "8f7e4b30056b33ab373baed5e3088955")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<add_way_point-response>)))
  "Returns full string definition for message of type '<add_way_point-response>"
  (cl:format cl:nil "bool recieved~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'add_way_point-response)))
  "Returns full string definition for message of type 'add_way_point-response"
  (cl:format cl:nil "bool recieved~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <add_way_point-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <add_way_point-response>))
  "Converts a ROS message object to a list"
  (cl:list 'add_way_point-response
    (cl:cons ':recieved (recieved msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'add_way_point)))
  'add_way_point-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'add_way_point)))
  'add_way_point-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'add_way_point)))
  "Returns string type for a service object of type '<add_way_point>"
  "way_point_wamv/add_way_point")