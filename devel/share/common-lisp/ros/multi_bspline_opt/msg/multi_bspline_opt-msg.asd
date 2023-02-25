
(cl:in-package :asdf)

(defsystem "multi_bspline_opt-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
<<<<<<< HEAD
               :std_msgs-msg
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
)
  :components ((:file "_package")
    (:file "BsplineTraj" :depends-on ("_package_BsplineTraj"))
    (:file "_package_BsplineTraj" :depends-on ("_package"))
    (:file "MultiBsplines" :depends-on ("_package_MultiBsplines"))
    (:file "_package_MultiBsplines" :depends-on ("_package"))
    (:file "SendTraj" :depends-on ("_package_SendTraj"))
    (:file "_package_SendTraj" :depends-on ("_package"))
  ))