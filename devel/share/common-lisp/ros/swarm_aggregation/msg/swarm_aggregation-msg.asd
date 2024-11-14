
(cl:in-package :asdf)

(defsystem "swarm_aggregation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "bot" :depends-on ("_package_bot"))
    (:file "_package_bot" :depends-on ("_package"))
    (:file "botPose" :depends-on ("_package_botPose"))
    (:file "_package_botPose" :depends-on ("_package"))
    (:file "obs" :depends-on ("_package_obs"))
    (:file "_package_obs" :depends-on ("_package"))
  ))