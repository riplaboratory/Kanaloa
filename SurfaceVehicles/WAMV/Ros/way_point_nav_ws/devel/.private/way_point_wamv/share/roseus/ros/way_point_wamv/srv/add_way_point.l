;; Auto-generated. Do not edit!


(when (boundp 'way_point_wamv::add_way_point)
  (if (not (find-package "WAY_POINT_WAMV"))
    (make-package "WAY_POINT_WAMV"))
  (shadow 'add_way_point (find-package "WAY_POINT_WAMV")))
(unless (find-package "WAY_POINT_WAMV::ADD_WAY_POINT")
  (make-package "WAY_POINT_WAMV::ADD_WAY_POINT"))
(unless (find-package "WAY_POINT_WAMV::ADD_WAY_POINTREQUEST")
  (make-package "WAY_POINT_WAMV::ADD_WAY_POINTREQUEST"))
(unless (find-package "WAY_POINT_WAMV::ADD_WAY_POINTRESPONSE")
  (make-package "WAY_POINT_WAMV::ADD_WAY_POINTRESPONSE"))

(in-package "ROS")





(defclass way_point_wamv::add_way_pointRequest
  :super ros::object
  :slots (_latitude _longitude _minutes ))

(defmethod way_point_wamv::add_way_pointRequest
  (:init
   (&key
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:minutes __minutes) 0.0)
    )
   (send-super :init)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _minutes (float __minutes))
   self)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:minutes
   (&optional __minutes)
   (if __minutes (setq _minutes __minutes)) _minutes)
  (:serialization-length
   ()
   (+
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _minutes
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _minutes
       (sys::poke _minutes (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _minutes
     (setq _minutes (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass way_point_wamv::add_way_pointResponse
  :super ros::object
  :slots (_recieved ))

(defmethod way_point_wamv::add_way_pointResponse
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

(defclass way_point_wamv::add_way_point
  :super ros::object
  :slots ())

(setf (get way_point_wamv::add_way_point :md5sum-) "8f7e4b30056b33ab373baed5e3088955")
(setf (get way_point_wamv::add_way_point :datatype-) "way_point_wamv/add_way_point")
(setf (get way_point_wamv::add_way_point :request) way_point_wamv::add_way_pointRequest)
(setf (get way_point_wamv::add_way_point :response) way_point_wamv::add_way_pointResponse)

(defmethod way_point_wamv::add_way_pointRequest
  (:response () (instance way_point_wamv::add_way_pointResponse :init)))

(setf (get way_point_wamv::add_way_pointRequest :md5sum-) "8f7e4b30056b33ab373baed5e3088955")
(setf (get way_point_wamv::add_way_pointRequest :datatype-) "way_point_wamv/add_way_pointRequest")
(setf (get way_point_wamv::add_way_pointRequest :definition-)
      "float64 latitude
float64 longitude
float64 minutes
---
bool recieved

")

(setf (get way_point_wamv::add_way_pointResponse :md5sum-) "8f7e4b30056b33ab373baed5e3088955")
(setf (get way_point_wamv::add_way_pointResponse :datatype-) "way_point_wamv/add_way_pointResponse")
(setf (get way_point_wamv::add_way_pointResponse :definition-)
      "float64 latitude
float64 longitude
float64 minutes
---
bool recieved

")



(provide :way_point_wamv/add_way_point "8f7e4b30056b33ab373baed5e3088955")


