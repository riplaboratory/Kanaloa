;; Auto-generated. Do not edit!


(when (boundp 'way_point_wamv::way_point_cmd)
  (if (not (find-package "WAY_POINT_WAMV"))
    (make-package "WAY_POINT_WAMV"))
  (shadow 'way_point_cmd (find-package "WAY_POINT_WAMV")))
(unless (find-package "WAY_POINT_WAMV::WAY_POINT_CMD")
  (make-package "WAY_POINT_WAMV::WAY_POINT_CMD"))
(unless (find-package "WAY_POINT_WAMV::WAY_POINT_CMDREQUEST")
  (make-package "WAY_POINT_WAMV::WAY_POINT_CMDREQUEST"))
(unless (find-package "WAY_POINT_WAMV::WAY_POINT_CMDRESPONSE")
  (make-package "WAY_POINT_WAMV::WAY_POINT_CMDRESPONSE"))

(in-package "ROS")





(defclass way_point_wamv::way_point_cmdRequest
  :super ros::object
  :slots (_command ))

(defmethod way_point_wamv::way_point_cmdRequest
  (:init
   (&key
    ((:command __command) "")
    )
   (send-super :init)
   (setq _command (string __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass way_point_wamv::way_point_cmdResponse
  :super ros::object
  :slots (_recieved ))

(defmethod way_point_wamv::way_point_cmdResponse
  (:init
   (&key
    ((:recieved __recieved) nil)
    )
   (send-super :init)
   (setq _recieved __recieved)
   self)
  (:recieved
   (&optional __recieved)
   (if __recieved (setq _recieved __recieved)) _recieved)
  (:serialization-length
   ()
   (+
    ;; bool _recieved
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _recieved
       (if _recieved (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _recieved
     (setq _recieved (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass way_point_wamv::way_point_cmd
  :super ros::object
  :slots ())

(setf (get way_point_wamv::way_point_cmd :md5sum-) "9115d0e027e87d5bbec3f644195b57c4")
(setf (get way_point_wamv::way_point_cmd :datatype-) "way_point_wamv/way_point_cmd")
(setf (get way_point_wamv::way_point_cmd :request) way_point_wamv::way_point_cmdRequest)
(setf (get way_point_wamv::way_point_cmd :response) way_point_wamv::way_point_cmdResponse)

(defmethod way_point_wamv::way_point_cmdRequest
  (:response () (instance way_point_wamv::way_point_cmdResponse :init)))

(setf (get way_point_wamv::way_point_cmdRequest :md5sum-) "9115d0e027e87d5bbec3f644195b57c4")
(setf (get way_point_wamv::way_point_cmdRequest :datatype-) "way_point_wamv/way_point_cmdRequest")
(setf (get way_point_wamv::way_point_cmdRequest :definition-)
      "string command
---
bool recieved

")

(setf (get way_point_wamv::way_point_cmdResponse :md5sum-) "9115d0e027e87d5bbec3f644195b57c4")
(setf (get way_point_wamv::way_point_cmdResponse :datatype-) "way_point_wamv/way_point_cmdResponse")
(setf (get way_point_wamv::way_point_cmdResponse :definition-)
      "string command
---
bool recieved

")



(provide :way_point_wamv/way_point_cmd "9115d0e027e87d5bbec3f644195b57c4")


