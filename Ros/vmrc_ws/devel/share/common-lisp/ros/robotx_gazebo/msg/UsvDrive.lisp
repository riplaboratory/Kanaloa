; Auto-generated. Do not edit!


(cl:in-package robotx_gazebo-msg)


;//! \htmlinclude UsvDrive.msg.html

(cl:defclass <UsvDrive> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0))
)

(cl:defclass UsvDrive (<UsvDrive>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UsvDrive>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UsvDrive)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotx_gazebo-msg:<UsvDrive> is deprecated: use robotx_gazebo-msg:UsvDrive instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <UsvDrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotx_gazebo-msg:left-val is deprecated.  Use robotx_gazebo-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <UsvDrive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotx_gazebo-msg:right-val is deprecated.  Use robotx_gazebo-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UsvDrive>) ostream)
  "Serializes a message object of type '<UsvDrive>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UsvDrive>) istream)
  "Deserializes a message object of type '<UsvDrive>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UsvDrive>)))
  "Returns string type for a message object of type '<UsvDrive>"
  "robotx_gazebo/UsvDrive")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UsvDrive)))
  "Returns string type for a message object of type 'UsvDrive"
  "robotx_gazebo/UsvDrive")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UsvDrive>)))
  "Returns md5sum for a message object of type '<UsvDrive>"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UsvDrive)))
  "Returns md5sum for a message object of type 'UsvDrive"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UsvDrive>)))
  "Returns full string definition for message of type '<UsvDrive>"
  (cl:format cl:nil "# Standard USV thrust massage - port and starbard~%~%# Thrust command - typically ranges from {-1.0 - 1.0}~%float32 left~%float32 right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UsvDrive)))
  "Returns full string definition for message of type 'UsvDrive"
  (cl:format cl:nil "# Standard USV thrust massage - port and starbard~%~%# Thrust command - typically ranges from {-1.0 - 1.0}~%float32 left~%float32 right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UsvDrive>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UsvDrive>))
  "Converts a ROS message object to a list"
  (cl:list 'UsvDrive
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
