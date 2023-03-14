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
  :slots (_drone_id _traj_id _order _cps_num_ _start_time _start_pos_ _start_vel_ _start_acc_ _end_pos_ _control_pts _knots ))

(defmethod multi_bspline_opt::SendTraj
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:traj_id __traj_id) 0)
    ((:order __order) 0)
    ((:cps_num_ __cps_num_) 0)
    ((:start_time __start_time) (instance ros::time :init))
    ((:start_pos_ __start_pos_) (instance geometry_msgs::Point :init))
    ((:start_vel_ __start_vel_) (instance geometry_msgs::Point :init))
    ((:start_acc_ __start_acc_) (instance geometry_msgs::Point :init))
    ((:end_pos_ __end_pos_) (instance geometry_msgs::Point :init))
    ((:control_pts __control_pts) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:knots __knots) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _traj_id (round __traj_id))
   (setq _order (round __order))
   (setq _cps_num_ (round __cps_num_))
   (setq _start_time __start_time)
   (setq _start_pos_ __start_pos_)
   (setq _start_vel_ __start_vel_)
   (setq _start_acc_ __start_acc_)
   (setq _end_pos_ __end_pos_)
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
  (:start_pos_
   (&rest __start_pos_)
   (if (keywordp (car __start_pos_))
       (send* _start_pos_ __start_pos_)
     (progn
       (if __start_pos_ (setq _start_pos_ (car __start_pos_)))
       _start_pos_)))
  (:start_vel_
   (&rest __start_vel_)
   (if (keywordp (car __start_vel_))
       (send* _start_vel_ __start_vel_)
     (progn
       (if __start_vel_ (setq _start_vel_ (car __start_vel_)))
       _start_vel_)))
  (:start_acc_
   (&rest __start_acc_)
   (if (keywordp (car __start_acc_))
       (send* _start_acc_ __start_acc_)
     (progn
       (if __start_acc_ (setq _start_acc_ (car __start_acc_)))
       _start_acc_)))
  (:end_pos_
   (&rest __end_pos_)
   (if (keywordp (car __end_pos_))
       (send* _end_pos_ __end_pos_)
     (progn
       (if __end_pos_ (setq _end_pos_ (car __end_pos_)))
       _end_pos_)))
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
    ;; geometry_msgs/Point _start_pos_
    (send _start_pos_ :serialization-length)
    ;; geometry_msgs/Point _start_vel_
    (send _start_vel_ :serialization-length)
    ;; geometry_msgs/Point _start_acc_
    (send _start_acc_ :serialization-length)
    ;; geometry_msgs/Point _end_pos_
    (send _end_pos_ :serialization-length)
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
     ;; geometry_msgs/Point _start_pos_
       (send _start_pos_ :serialize s)
     ;; geometry_msgs/Point _start_vel_
       (send _start_vel_ :serialize s)
     ;; geometry_msgs/Point _start_acc_
       (send _start_acc_ :serialize s)
     ;; geometry_msgs/Point _end_pos_
       (send _end_pos_ :serialize s)
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
   ;; geometry_msgs/Point _start_pos_
     (send _start_pos_ :deserialize buf ptr-) (incf ptr- (send _start_pos_ :serialization-length))
   ;; geometry_msgs/Point _start_vel_
     (send _start_vel_ :deserialize buf ptr-) (incf ptr- (send _start_vel_ :serialization-length))
   ;; geometry_msgs/Point _start_acc_
     (send _start_acc_ :deserialize buf ptr-) (incf ptr- (send _start_acc_ :serialization-length))
   ;; geometry_msgs/Point _end_pos_
     (send _end_pos_ :deserialize buf ptr-) (incf ptr- (send _end_pos_ :serialization-length))
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

(setf (get multi_bspline_opt::SendTraj :md5sum-) "d1c07ca91141848b23c4fb9cb569a4f8")
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


geometry_msgs/Point start_pos_
geometry_msgs/Point start_vel_
geometry_msgs/Point start_acc_
geometry_msgs/Point end_pos_
# float64 start_pos_x
# float64 start_pos_y
# float64 start_vel_x
# float64 start_vel_y
# float64 start_acc_x
# float64 start_acc_y
# float64 end_pos_x
# float64 end_pos_y
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



(provide :multi_bspline_opt/SendTraj "d1c07ca91141848b23c4fb9cb569a4f8")


