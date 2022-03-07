;; Auto-generated. Do not edit!


(when (boundp 'yolact_ros_msgs::Detection)
  (if (not (find-package "YOLACT_ROS_MSGS"))
    (make-package "YOLACT_ROS_MSGS"))
  (shadow 'Detection (find-package "YOLACT_ROS_MSGS")))
(unless (find-package "YOLACT_ROS_MSGS::DETECTION")
  (make-package "YOLACT_ROS_MSGS::DETECTION"))

(in-package "ROS")
;;//! \htmlinclude Detection.msg.html


(defclass yolact_ros_msgs::Detection
  :super ros::object
  :slots (_class_name _score _box _mask ))

(defmethod yolact_ros_msgs::Detection
  (:init
   (&key
    ((:class_name __class_name) "")
    ((:score __score) 0.0)
    ((:box __box) (instance yolact_ros_msgs::Box :init))
    ((:mask __mask) (instance yolact_ros_msgs::Mask :init))
    )
   (send-super :init)
   (setq _class_name (string __class_name))
   (setq _score (float __score))
   (setq _box __box)
   (setq _mask __mask)
   self)
  (:class_name
   (&optional __class_name)
   (if __class_name (setq _class_name __class_name)) _class_name)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:box
   (&rest __box)
   (if (keywordp (car __box))
       (send* _box __box)
     (progn
       (if __box (setq _box (car __box)))
       _box)))
  (:mask
   (&rest __mask)
   (if (keywordp (car __mask))
       (send* _mask __mask)
     (progn
       (if __mask (setq _mask (car __mask)))
       _mask)))
  (:serialization-length
   ()
   (+
    ;; string _class_name
    4 (length _class_name)
    ;; float64 _score
    8
    ;; yolact_ros_msgs/Box _box
    (send _box :serialization-length)
    ;; yolact_ros_msgs/Mask _mask
    (send _mask :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _class_name
       (write-long (length _class_name) s) (princ _class_name s)
     ;; float64 _score
       (sys::poke _score (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; yolact_ros_msgs/Box _box
       (send _box :serialize s)
     ;; yolact_ros_msgs/Mask _mask
       (send _mask :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _class_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _class_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _score
     (setq _score (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; yolact_ros_msgs/Box _box
     (send _box :deserialize buf ptr-) (incf ptr- (send _box :serialization-length))
   ;; yolact_ros_msgs/Mask _mask
     (send _mask :deserialize buf ptr-) (incf ptr- (send _mask :serialization-length))
   ;;
   self)
  )

(setf (get yolact_ros_msgs::Detection :md5sum-) "b9e8252cb1b190158c1553f6547a1db7")
(setf (get yolact_ros_msgs::Detection :datatype-) "yolact_ros_msgs/Detection")
(setf (get yolact_ros_msgs::Detection :definition-)
      "string class_name
float64 score
Box box
Mask mask

================================================================================
MSG: yolact_ros_msgs/Box
int32 x1
int32 y1
int32 x2
int32 y2

================================================================================
MSG: yolact_ros_msgs/Mask
int32 width
int32 height
uint8[] mask # Mask encoded as bitset

")



(provide :yolact_ros_msgs/Detection "b9e8252cb1b190158c1553f6547a1db7")


