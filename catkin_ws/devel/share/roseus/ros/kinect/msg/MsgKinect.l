;; Auto-generated. Do not edit!


(when (boundp 'kinect::MsgKinect)
  (if (not (find-package "KINECT"))
    (make-package "KINECT"))
  (shadow 'MsgKinect (find-package "KINECT")))
(unless (find-package "KINECT::MSGKINECT")
  (make-package "KINECT::MSGKINECT"))

(in-package "ROS")
;;//! \htmlinclude MsgKinect.msg.html


(defclass kinect::MsgKinect
  :super ros::object
  :slots (_data ))

(defmethod kinect::MsgKinect
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _data
       (write-long _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kinect::MsgKinect :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get kinect::MsgKinect :datatype-) "kinect/MsgKinect")
(setf (get kinect::MsgKinect :definition-)
      "int32 data

")



(provide :kinect/MsgKinect "da5909fbe378aeaf85e547e830cc1bb7")


