;; Auto-generated. Do not edit!


(when (boundp 'multi_bspline_opt::BsplineTraj)
  (if (not (find-package "MULTI_BSPLINE_OPT"))
    (make-package "MULTI_BSPLINE_OPT"))
  (shadow 'BsplineTraj (find-package "MULTI_BSPLINE_OPT")))
(unless (find-package "MULTI_BSPLINE_OPT::BSPLINETRAJ")
  (make-package "MULTI_BSPLINE_OPT::BSPLINETRAJ"))

(in-package "ROS")
;;//! \htmlinclude BsplineTraj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
<<<<<<< HEAD
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd


(defclass multi_bspline_opt::BsplineTraj
  :super ros::object
<<<<<<< HEAD
  :slots (_header _position _velocity _acceleration _yaw _yaw_rate _current_seq ))
=======
  :slots (_drone_id _traj_id _start_time _duration _yaw _yaw_rate _position _velocity _acceleration _current_seq ))
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd

(defmethod multi_bspline_opt::BsplineTraj
  (:init
   (&key
<<<<<<< HEAD
    ((:header __header) (instance std_msgs::Header :init))
    ((:position __position) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:velocity __velocity) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:acceleration __acceleration) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:yaw __yaw) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:current_seq __current_seq) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   (setq _yaw (float __yaw))
   (setq _yaw_rate (float __yaw_rate))
   (setq _current_seq (round __current_seq))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
=======
    ((:drone_id __drone_id) 0)
    ((:traj_id __traj_id) 0)
    ((:start_time __start_time) (instance ros::time :init))
    ((:duration __duration) 0.0)
    ((:yaw __yaw) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:position __position) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:velocity __velocity) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:acceleration __acceleration) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:current_seq __current_seq) 0)
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _traj_id (round __traj_id))
   (setq _start_time __start_time)
   (setq _duration (float __duration))
   (setq _yaw (float __yaw))
   (setq _yaw_rate (float __yaw_rate))
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   (setq _current_seq (round __current_seq))
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:traj_id
   (&optional __traj_id)
   (if __traj_id (setq _traj_id __traj_id)) _traj_id)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
<<<<<<< HEAD
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
  (:current_seq
   (&optional __current_seq)
   (if __current_seq (setq _current_seq __current_seq)) _current_seq)
  (:serialization-length
   ()
   (+
<<<<<<< HEAD
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/PoseStamped[] _position
    (apply #'+ (send-all _position :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _velocity
    (apply #'+ (send-all _velocity :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _acceleration
    (apply #'+ (send-all _acceleration :serialization-length)) 4
    ;; float32 _yaw
    4
    ;; float32 _yaw_rate
    4
=======
    ;; int32 _drone_id
    4
    ;; int64 _traj_id
    8
    ;; time _start_time
    8
    ;; float64 _duration
    8
    ;; float64 _yaw
    8
    ;; float64 _yaw_rate
    8
    ;; geometry_msgs/Point[] _position
    (apply #'+ (send-all _position :serialization-length)) 4
    ;; geometry_msgs/Point[] _velocity
    (apply #'+ (send-all _velocity :serialization-length)) 4
    ;; geometry_msgs/Point[] _acceleration
    (apply #'+ (send-all _acceleration :serialization-length)) 4
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
    ;; uint32 _current_seq
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
<<<<<<< HEAD
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/PoseStamped[] _position
=======
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
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; float64 _duration
       (sys::poke _duration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Point[] _position
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
     (write-long (length _position) s)
     (dolist (elem _position)
       (send elem :serialize s)
       )
<<<<<<< HEAD
     ;; geometry_msgs/PoseStamped[] _velocity
=======
     ;; geometry_msgs/Point[] _velocity
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
     (write-long (length _velocity) s)
     (dolist (elem _velocity)
       (send elem :serialize s)
       )
<<<<<<< HEAD
     ;; geometry_msgs/PoseStamped[] _acceleration
=======
     ;; geometry_msgs/Point[] _acceleration
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
     (write-long (length _acceleration) s)
     (dolist (elem _acceleration)
       (send elem :serialize s)
       )
<<<<<<< HEAD
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
     ;; uint32 _current_seq
       (write-long _current_seq s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
<<<<<<< HEAD
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/PoseStamped[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _position)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _velocity)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _acceleration
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _acceleration (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _acceleration)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
=======
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
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _duration
     (setq _duration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Point[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _position)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _velocity)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _acceleration
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _acceleration (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _acceleration)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
   ;; uint32 _current_seq
     (setq _current_seq (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

<<<<<<< HEAD
(setf (get multi_bspline_opt::BsplineTraj :md5sum-) "47fe784cc6ad92a6cd7d1488e4d545d9")
(setf (get multi_bspline_opt::BsplineTraj :datatype-) "multi_bspline_opt/BsplineTraj")
(setf (get multi_bspline_opt::BsplineTraj :definition-)
      "std_msgs/Header header

geometry_msgs/PoseStamped[] position
geometry_msgs/PoseStamped[] velocity
geometry_msgs/PoseStamped[] acceleration

float32 yaw
float32 yaw_rate
uint32 current_seq

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation
=======
(setf (get multi_bspline_opt::BsplineTraj :md5sum-) "c277c0fe692a3c44cb39de5d41a0fe4c")
(setf (get multi_bspline_opt::BsplineTraj :datatype-) "multi_bspline_opt/BsplineTraj")
(setf (get multi_bspline_opt::BsplineTraj :definition-)
      "# std_msgs/Header header

int32 drone_id
int64 traj_id
time start_time
float64 duration

float64 yaw
float64 yaw_rate

geometry_msgs/Point[] position
geometry_msgs/Point[] velocity
geometry_msgs/Point[] acceleration

uint32 current_seq
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

<<<<<<< HEAD
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

=======
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
")



<<<<<<< HEAD
(provide :multi_bspline_opt/BsplineTraj "47fe784cc6ad92a6cd7d1488e4d545d9")
=======
(provide :multi_bspline_opt/BsplineTraj "c277c0fe692a3c44cb39de5d41a0fe4c")
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd


