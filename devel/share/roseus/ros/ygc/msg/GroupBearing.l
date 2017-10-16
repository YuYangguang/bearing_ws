;; Auto-generated. Do not edit!


(when (boundp 'ygc::GroupBearing)
  (if (not (find-package "YGC"))
    (make-package "YGC"))
  (shadow 'GroupBearing (find-package "YGC")))
(unless (find-package "YGC::GROUPBEARING")
  (make-package "YGC::GROUPBEARING"))

(in-package "ROS")
;;//! \htmlinclude GroupBearing.msg.html


(defclass ygc::GroupBearing
  :super ros::object
  :slots (_bearings ))

(defmethod ygc::GroupBearing
  (:init
   (&key
    ((:bearings __bearings) (let (r) (dotimes (i 0) (push (instance ygc::Bearing2D :init) r)) r))
    )
   (send-super :init)
   (setq _bearings __bearings)
   self)
  (:bearings
   (&rest __bearings)
   (if (keywordp (car __bearings))
       (send* _bearings __bearings)
     (progn
       (if __bearings (setq _bearings (car __bearings)))
       _bearings)))
  (:serialization-length
   ()
   (+
    ;; ygc/Bearing2D[] _bearings
    (apply #'+ (send-all _bearings :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ygc/Bearing2D[] _bearings
     (write-long (length _bearings) s)
     (dolist (elem _bearings)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ygc/Bearing2D[] _bearings
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bearings (let (r) (dotimes (i n) (push (instance ygc::Bearing2D :init) r)) r))
     (dolist (elem- _bearings)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ygc::GroupBearing :md5sum-) "f03b323f78db40d62b3828c934a95919")
(setf (get ygc::GroupBearing :datatype-) "ygc/GroupBearing")
(setf (get ygc::GroupBearing :definition-)
      "Bearing2D[] bearings

================================================================================
MSG: ygc/Bearing2D
float32 x
float32 y

")



(provide :ygc/GroupBearing "f03b323f78db40d62b3828c934a95919")


