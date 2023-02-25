;; Auto-generated. Do not edit!


(when (boundp 'multi_bspline_opt::MultiBsplines)
  (if (not (find-package "MULTI_BSPLINE_OPT"))
    (make-package "MULTI_BSPLINE_OPT"))
  (shadow 'MultiBsplines (find-package "MULTI_BSPLINE_OPT")))
(unless (find-package "MULTI_BSPLINE_OPT::MULTIBSPLINES")
  (make-package "MULTI_BSPLINE_OPT::MULTIBSPLINES"))

(in-package "ROS")
;;//! \htmlinclude MultiBsplines.msg.html


(defclass multi_bspline_opt::MultiBsplines
  :super ros::object
  :slots (_drone_id_from _traj ))

(defmethod multi_bspline_opt::MultiBsplines
  (:init
   (&key
    ((:drone_id_from __drone_id_from) 0)
    ((:traj __traj) (let (r) (dotimes (i 0) (push (instance multi_bspline_opt::SendTraj :init) r)) r))
    )
   (send-super :init)
   (setq _drone_id_from (round __drone_id_from))
   (setq _traj __traj)
   self)
  (:drone_id_from
   (&optional __drone_id_from)
   (if __drone_id_from (setq _drone_id_from __drone_id_from)) _drone_id_from)
  (:traj
   (&rest __traj)
   (if (keywordp (car __traj))
       (send* _traj __traj)
     (progn
       (if __traj (setq _traj (car __traj)))
       _traj)))
  (:serialization-length
   ()
   (+
    ;; int32 _drone_id_from
    4
    ;; multi_bspline_opt/SendTraj[] _traj
    (apply #'+ (send-all _traj :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _drone_id_from
       (write-long _drone_id_from s)
     ;; multi_bspline_opt/SendTraj[] _traj
     (write-long (length _traj) s)
     (dolist (elem _traj)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _drone_id_from
     (setq _drone_id_from (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; multi_bspline_opt/SendTraj[] _traj
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _traj (let (r) (dotimes (i n) (push (instance multi_bspline_opt::SendTraj :init) r)) r))
     (dolist (elem- _traj)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get multi_bspline_opt::MultiBsplines :md5sum-) "0a9ce0a6a663879c244f8d58ac09d0d8")
(setf (get multi_bspline_opt::MultiBsplines :datatype-) "multi_bspline_opt/MultiBsplines")
(setf (get multi_bspline_opt::MultiBsplines :definition-)
      "int32 drone_id_from

SendTraj[] traj
================================================================================
MSG: multi_bspline_opt/SendTraj
int32 drone_id
int64 traj_id
int32 order
int32 cps_num_
# int32 Dim_
# int32 TrajSampleRate
# float64 beta
time start_time


float64 start_pos_x
float64 start_pos_y
float64 start_vel_x
float64 start_vel_y
float64 end_pos_x
float64 end_pos_y
# float64 yaw_rate

geometry_msgs/Point[] control_pts
float64[] knots
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :multi_bspline_opt/MultiBsplines "0a9ce0a6a663879c244f8d58ac09d0d8")


