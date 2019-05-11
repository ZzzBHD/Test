; Auto-generated. Do not edit!


(cl:in-package kinect-msg)


;//! \htmlinclude MsgKinect.msg.html

(cl:defclass <MsgKinect> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass MsgKinect (<MsgKinect>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MsgKinect>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MsgKinect)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kinect-msg:<MsgKinect> is deprecated: use kinect-msg:MsgKinect instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <MsgKinect>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kinect-msg:data-val is deprecated.  Use kinect-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MsgKinect>) ostream)
  "Serializes a message object of type '<MsgKinect>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MsgKinect>) istream)
  "Deserializes a message object of type '<MsgKinect>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MsgKinect>)))
  "Returns string type for a message object of type '<MsgKinect>"
  "kinect/MsgKinect")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MsgKinect)))
  "Returns string type for a message object of type 'MsgKinect"
  "kinect/MsgKinect")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MsgKinect>)))
  "Returns md5sum for a message object of type '<MsgKinect>"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MsgKinect)))
  "Returns md5sum for a message object of type 'MsgKinect"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MsgKinect>)))
  "Returns full string definition for message of type '<MsgKinect>"
  (cl:format cl:nil "int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MsgKinect)))
  "Returns full string definition for message of type 'MsgKinect"
  (cl:format cl:nil "int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MsgKinect>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MsgKinect>))
  "Converts a ROS message object to a list"
  (cl:list 'MsgKinect
    (cl:cons ':data (data msg))
))
