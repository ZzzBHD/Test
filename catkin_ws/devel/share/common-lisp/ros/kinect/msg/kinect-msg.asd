
(cl:in-package :asdf)

(defsystem "kinect-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MsgKinect" :depends-on ("_package_MsgKinect"))
    (:file "_package_MsgKinect" :depends-on ("_package"))
  ))