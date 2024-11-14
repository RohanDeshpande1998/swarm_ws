;; Auto-generated. Do not edit!


(when (boundp 'swarm_aggregation::bot)
  (if (not (find-package "SWARM_AGGREGATION"))
    (make-package "SWARM_AGGREGATION"))
  (shadow 'bot (find-package "SWARM_AGGREGATION")))
(unless (find-package "SWARM_AGGREGATION::BOT")
  (make-package "SWARM_AGGREGATION::BOT"))

(in-package "ROS")
;;//! \htmlinclude bot.msg.html


(defclass swarm_aggregation::bot
  :super ros::object
  :slots (_id _pose_x _pose_y _pose_theta _vel_x _vel_y ))

(defmethod swarm_aggregation::bot
  (:init
   (&key
    ((:id __id) "")
    ((:pose_x __pose_x) 0.0)
    ((:pose_y __pose_y) 0.0)
    ((:pose_theta __pose_theta) 0.0)
    ((:vel_x __vel_x) 0.0)
    ((:vel_y __vel_y) 0.0)
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _pose_x (float __pose_x))
   (setq _pose_y (float __pose_y))
   (setq _pose_theta (float __pose_theta))
   (setq _vel_x (float __vel_x))
   (setq _vel_y (float __vel_y))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pose_x
   (&optional __pose_x)
   (if __pose_x (setq _pose_x __pose_x)) _pose_x)
  (:pose_y
   (&optional __pose_y)
   (if __pose_y (setq _pose_y __pose_y)) _pose_y)
  (:pose_theta
   (&optional __pose_theta)
   (if __pose_theta (setq _pose_theta __pose_theta)) _pose_theta)
  (:vel_x
   (&optional __vel_x)
   (if __vel_x (setq _vel_x __vel_x)) _vel_x)
  (:vel_y
   (&optional __vel_y)
   (if __vel_y (setq _vel_y __vel_y)) _vel_y)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; float64 _pose_x
    8
    ;; float64 _pose_y
    8
    ;; float64 _pose_theta
    8
    ;; float64 _vel_x
    8
    ;; float64 _vel_y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; float64 _pose_x
       (sys::poke _pose_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pose_y
       (sys::poke _pose_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pose_theta
       (sys::poke _pose_theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_x
       (sys::poke _vel_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_y
       (sys::poke _vel_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _pose_x
     (setq _pose_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pose_y
     (setq _pose_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pose_theta
     (setq _pose_theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_x
     (setq _vel_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_y
     (setq _vel_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get swarm_aggregation::bot :md5sum-) "4cb3c0cf91cbe6f044efb99c3a3f8cdf")
(setf (get swarm_aggregation::bot :datatype-) "swarm_aggregation/bot")
(setf (get swarm_aggregation::bot :definition-)
      "string id
float64 pose_x
float64 pose_y
float64 pose_theta
float64 vel_x
float64 vel_y

")



(provide :swarm_aggregation/bot "4cb3c0cf91cbe6f044efb99c3a3f8cdf")


