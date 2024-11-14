;; Auto-generated. Do not edit!


(when (boundp 'swarm_aggregation::botPose)
  (if (not (find-package "SWARM_AGGREGATION"))
    (make-package "SWARM_AGGREGATION"))
  (shadow 'botPose (find-package "SWARM_AGGREGATION")))
(unless (find-package "SWARM_AGGREGATION::BOTPOSE")
  (make-package "SWARM_AGGREGATION::BOTPOSE"))

(in-package "ROS")
;;//! \htmlinclude botPose.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass swarm_aggregation::botPose
  :super ros::object
  :slots (_bot_id _botpose ))

(defmethod swarm_aggregation::botPose
  (:init
   (&key
    ((:bot_id __bot_id) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:botpose __botpose) ())
    )
   (send-super :init)
   (setq _bot_id __bot_id)
   (setq _botpose __botpose)
   self)
  (:bot_id
   (&optional __bot_id)
   (if __bot_id (setq _bot_id __bot_id)) _bot_id)
  (:botpose
   (&rest __botpose)
   (if (keywordp (car __botpose))
       (send* _botpose __botpose)
     (progn
       (if __botpose (setq _botpose (car __botpose)))
       _botpose)))
  (:serialization-length
   ()
   (+
    ;; string[] _bot_id
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _bot_id)) 4
    ;; nav_msgs/Odometry[] _botpose
    (apply #'+ (send-all _botpose :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _bot_id
     (write-long (length _bot_id) s)
     (dolist (elem _bot_id)
       (write-long (length elem) s) (princ elem s)
       )
     ;; nav_msgs/Odometry[] _botpose
     (write-long (length _botpose) s)
     (dolist (elem _botpose)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _bot_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bot_id (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _bot_id i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; nav_msgs/Odometry[] _botpose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _botpose (let (r) (dotimes (i n) (push (instance nav_msgs::Odometry :init) r)) r))
     (dolist (elem- _botpose)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get swarm_aggregation::botPose :md5sum-) "14f9cb1d05d7b333efc6809331fe9a60")
(setf (get swarm_aggregation::botPose :datatype-) "swarm_aggregation/botPose")
(setf (get swarm_aggregation::botPose :definition-)
      "string[] bot_id
nav_msgs/Odometry[] botpose

================================================================================
MSG: nav_msgs/Odometry
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :swarm_aggregation/botPose "14f9cb1d05d7b333efc6809331fe9a60")


