;; Auto-generated. Do not edit!


(when (boundp 'multi_bspline_opt::SendTraj)
  (if (not (find-package "MULTI_BSPLINE_OPT"))
    (make-package "MULTI_BSPLINE_OPT"))
  (shadow 'SendTraj (find-package "MULTI_BSPLINE_OPT")))
(unless (find-package "MULTI_BSPLINE_OPT::SENDTRAJ")
  (make-package "MULTI_BSPLINE_OPT::SENDTRAJ"))

(in-package "ROS")
;;//! \htmlinclude SendTraj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass multi_bspline_opt::SendTraj
  :super ros::object
<<<<<<< HEAD
  :slots (_drone_id _traj_id _order _cps_num_ _start_time _start_pos_x _start_pos_y _start_vel_x _start_vel_y _start_acc_x _start_acc_y _end_pos_x _end_pos_y _control_pts _knots ))
=======
  :slots (_drone_id _traj_id _order _cps_num_ _start_time _start_pos_x _start_pos_y _start_vel_x _start_vel_y _end_pos_x _end_pos_y _control_pts _knots ))
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd

(defmethod multi_bspline_opt::SendTraj
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:traj_id __traj_id) 0)
    ((:order __order) 0)
    ((:cps_num_ __cps_num_) 0)
    ((:start_time __start_time) (instance ros::time :init))
    ((:start_pos_x __start_pos_x) 0.0)
    ((:start_pos_y __start_pos_y) 0.0)
    ((:start_vel_x __start_vel_x) 0.0)
    ((:start_vel_y __start_vel_y) 0.0)
<<<<<<< HEAD
    ((:start_acc_x __start_acc_x) 0.0)
    ((:start_acc_y __start_acc_y) 0.0)
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    ((:end_pos_x __end_pos_x) 0.0)
    ((:end_pos_y __end_pos_y) 0.0)
    ((:control_pts __control_pts) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:knots __knots) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _traj_id (round __traj_id))
   (setq _order (round __order))
   (setq _cps_num_ (round __cps_num_))
   (setq _start_time __start_time)
   (setq _start_pos_x (float __start_pos_x))
   (setq _start_pos_y (float __start_pos_y))
   (setq _start_vel_x (float __start_vel_x))
   (setq _start_vel_y (float __start_vel_y))
<<<<<<< HEAD
   (setq _start_acc_x (float __start_acc_x))
   (setq _start_acc_y (float __start_acc_y))
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
   (setq _end_pos_x (float __end_pos_x))
   (setq _end_pos_y (float __end_pos_y))
   (setq _control_pts __control_pts)
   (setq _knots __knots)
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:traj_id
   (&optional __traj_id)
   (if __traj_id (setq _traj_id __traj_id)) _traj_id)
  (:order
   (&optional __order)
   (if __order (setq _order __order)) _order)
  (:cps_num_
   (&optional __cps_num_)
   (if __cps_num_ (setq _cps_num_ __cps_num_)) _cps_num_)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:start_pos_x
   (&optional __start_pos_x)
   (if __start_pos_x (setq _start_pos_x __start_pos_x)) _start_pos_x)
  (:start_pos_y
   (&optional __start_pos_y)
   (if __start_pos_y (setq _start_pos_y __start_pos_y)) _start_pos_y)
  (:start_vel_x
   (&optional __start_vel_x)
   (if __start_vel_x (setq _start_vel_x __start_vel_x)) _start_vel_x)
  (:start_vel_y
   (&optional __start_vel_y)
   (if __start_vel_y (setq _start_vel_y __start_vel_y)) _start_vel_y)
<<<<<<< HEAD
  (:start_acc_x
   (&optional __start_acc_x)
   (if __start_acc_x (setq _start_acc_x __start_acc_x)) _start_acc_x)
  (:start_acc_y
   (&optional __start_acc_y)
   (if __start_acc_y (setq _start_acc_y __start_acc_y)) _start_acc_y)
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
  (:end_pos_x
   (&optional __end_pos_x)
   (if __end_pos_x (setq _end_pos_x __end_pos_x)) _end_pos_x)
  (:end_pos_y
   (&optional __end_pos_y)
   (if __end_pos_y (setq _end_pos_y __end_pos_y)) _end_pos_y)
  (:control_pts
   (&rest __control_pts)
   (if (keywordp (car __control_pts))
       (send* _control_pts __control_pts)
     (progn
       (if __control_pts (setq _control_pts (car __control_pts)))
       _control_pts)))
  (:knots
   (&optional __knots)
   (if __knots (setq _knots __knots)) _knots)
  (:serialization-length
   ()
   (+
    ;; int32 _drone_id
    4
    ;; int64 _traj_id
    8
    ;; int32 _order
    4
    ;; int32 _cps_num_
    4
    ;; time _start_time
    8
    ;; float64 _start_pos_x
    8
    ;; float64 _start_pos_y
    8
    ;; float64 _start_vel_x
    8
    ;; float64 _start_vel_y
    8
<<<<<<< HEAD
    ;; float64 _start_acc_x
    8
    ;; float64 _start_acc_y
    8
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    ;; float64 _end_pos_x
    8
    ;; float64 _end_pos_y
    8
    ;; geometry_msgs/Point[] _control_pts
    (apply #'+ (send-all _control_pts :serialization-length)) 4
    ;; float64[] _knots
    (* 8    (length _knots)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _drone_id
       (write-long _drone_id s)
     ;; int64 _traj_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _traj_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _traj_id) (= (length (_traj_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_traj_id . bv) 0) 0) s)
              (write-long (ash (elt (_traj_id . bv) 1) -1) s))
             ((and (class _traj_id) (= (length (_traj_id . bv)) 1)) ;; big1
              (write-long (elt (_traj_id . bv) 0) s)
              (write-long (if (>= _traj_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _traj_id s)(write-long (if (>= _traj_id 0) 0 #xffffffff) s)))
     ;; int32 _order
       (write-long _order s)
     ;; int32 _cps_num_
       (write-long _cps_num_ s)
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; float64 _start_pos_x
       (sys::poke _start_pos_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _start_pos_y
       (sys::poke _start_pos_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _start_vel_x
       (sys::poke _start_vel_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _start_vel_y
       (sys::poke _start_vel_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
<<<<<<< HEAD
     ;; float64 _start_acc_x
       (sys::poke _start_acc_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _start_acc_y
       (sys::poke _start_acc_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
     ;; float64 _end_pos_x
       (sys::poke _end_pos_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _end_pos_y
       (sys::poke _end_pos_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point[] _control_pts
     (write-long (length _control_pts) s)
     (dolist (elem _control_pts)
       (send elem :serialize s)
       )
     ;; float64[] _knots
     (write-long (length _knots) s)
     (dotimes (i (length _knots))
       (sys::poke (elt _knots i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _drone_id
     (setq _drone_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _traj_id
#+(or :alpha :irix6 :x86_64)
      (setf _traj_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _traj_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int32 _order
     (setq _order (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _cps_num_
     (setq _cps_num_ (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _start_pos_x
     (setq _start_pos_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _start_pos_y
     (setq _start_pos_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _start_vel_x
     (setq _start_vel_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _start_vel_y
     (setq _start_vel_y (sys::peek buf ptr- :double)) (incf ptr- 8)
<<<<<<< HEAD
   ;; float64 _start_acc_x
     (setq _start_acc_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _start_acc_y
     (setq _start_acc_y (sys::peek buf ptr- :double)) (incf ptr- 8)
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
   ;; float64 _end_pos_x
     (setq _end_pos_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _end_pos_y
     (setq _end_pos_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point[] _control_pts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _control_pts (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _control_pts)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _knots
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _knots (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _knots i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

<<<<<<< HEAD
(setf (get multi_bspline_opt::SendTraj :md5sum-) "1db39cd31304eba253abaf4c9988cb49")
=======
(setf (get multi_bspline_opt::SendTraj :md5sum-) "cdb4d98dc8568f5ace0ee75482493f76")
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
(setf (get multi_bspline_opt::SendTraj :datatype-) "multi_bspline_opt/SendTraj")
(setf (get multi_bspline_opt::SendTraj :definition-)
      "int32 drone_id
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
<<<<<<< HEAD
float64 start_acc_x
float64 start_acc_y
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
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



<<<<<<< HEAD
(provide :multi_bspline_opt/SendTraj "1db39cd31304eba253abaf4c9988cb49")
=======
(provide :multi_bspline_opt/SendTraj "cdb4d98dc8568f5ace0ee75482493f76")
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd


