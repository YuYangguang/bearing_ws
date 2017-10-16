;; Auto-generated. Do not edit!


(when (boundp 'ygc::Bearing2D)
  (if (not (find-package "YGC"))
    (make-package "YGC"))
  (shadow 'Bearing2D (find-package "YGC")))
(unless (find-package "YGC::BEARING2D")
  (make-package "YGC::BEARING2D"))

(in-package "ROS")
;;//! \htmlinclude Bearing2D.msg.html


(defclass ygc::Bearing2D
  :super ros::object
  :slots (_x _y ))

(defmethod ygc::Bearing2D
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ygc::Bearing2D :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get ygc::Bearing2D :datatype-) "ygc/Bearing2D")
(setf (get ygc::Bearing2D :definition-)
      "float32 x
float32 y

")



(provide :ygc/Bearing2D "ff8d7d66dd3e4b731ef14a45d38888b6")


