; Auto-generated. Do not edit!


(cl:in-package multi_bspline_opt-msg)


;//! \htmlinclude SendTraj.msg.html

(cl:defclass <SendTraj> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:integer
    :initform 0)
   (traj_id
    :reader traj_id
    :initarg :traj_id
    :type cl:integer
    :initform 0)
   (order
    :reader order
    :initarg :order
    :type cl:integer
    :initform 0)
   (cps_num_
    :reader cps_num_
    :initarg :cps_num_
    :type cl:integer
    :initform 0)
   (start_time
    :reader start_time
    :initarg :start_time
    :type cl:real
    :initform 0)
   (start_pos_
    :reader start_pos_
    :initarg :start_pos_
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (start_vel_
    :reader start_vel_
    :initarg :start_vel_
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (start_acc_
    :reader start_acc_
    :initarg :start_acc_
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (end_pos_
    :reader end_pos_
    :initarg :end_pos_
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (control_pts
    :reader control_pts
    :initarg :control_pts
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (knots
    :reader knots
    :initarg :knots
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SendTraj (<SendTraj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendTraj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendTraj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_bspline_opt-msg:<SendTraj> is deprecated: use multi_bspline_opt-msg:SendTraj instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:drone_id-val is deprecated.  Use multi_bspline_opt-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'traj_id-val :lambda-list '(m))
(cl:defmethod traj_id-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:traj_id-val is deprecated.  Use multi_bspline_opt-msg:traj_id instead.")
  (traj_id m))

(cl:ensure-generic-function 'order-val :lambda-list '(m))
(cl:defmethod order-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:order-val is deprecated.  Use multi_bspline_opt-msg:order instead.")
  (order m))

(cl:ensure-generic-function 'cps_num_-val :lambda-list '(m))
(cl:defmethod cps_num_-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:cps_num_-val is deprecated.  Use multi_bspline_opt-msg:cps_num_ instead.")
  (cps_num_ m))

(cl:ensure-generic-function 'start_time-val :lambda-list '(m))
(cl:defmethod start_time-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:start_time-val is deprecated.  Use multi_bspline_opt-msg:start_time instead.")
  (start_time m))

(cl:ensure-generic-function 'start_pos_-val :lambda-list '(m))
(cl:defmethod start_pos_-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:start_pos_-val is deprecated.  Use multi_bspline_opt-msg:start_pos_ instead.")
  (start_pos_ m))

(cl:ensure-generic-function 'start_vel_-val :lambda-list '(m))
(cl:defmethod start_vel_-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:start_vel_-val is deprecated.  Use multi_bspline_opt-msg:start_vel_ instead.")
  (start_vel_ m))

(cl:ensure-generic-function 'start_acc_-val :lambda-list '(m))
(cl:defmethod start_acc_-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:start_acc_-val is deprecated.  Use multi_bspline_opt-msg:start_acc_ instead.")
  (start_acc_ m))

(cl:ensure-generic-function 'end_pos_-val :lambda-list '(m))
(cl:defmethod end_pos_-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:end_pos_-val is deprecated.  Use multi_bspline_opt-msg:end_pos_ instead.")
  (end_pos_ m))

(cl:ensure-generic-function 'control_pts-val :lambda-list '(m))
(cl:defmethod control_pts-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:control_pts-val is deprecated.  Use multi_bspline_opt-msg:control_pts instead.")
  (control_pts m))

(cl:ensure-generic-function 'knots-val :lambda-list '(m))
(cl:defmethod knots-val ((m <SendTraj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:knots-val is deprecated.  Use multi_bspline_opt-msg:knots instead.")
  (knots m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendTraj>) ostream)
  "Serializes a message object of type '<SendTraj>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'traj_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'order)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cps_num_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'start_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'start_time) (cl:floor (cl:slot-value msg 'start_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_pos_) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_vel_) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_acc_) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end_pos_) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'control_pts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'control_pts))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'knots))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'knots))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendTraj>) istream)
  "Deserializes a message object of type '<SendTraj>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'traj_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'order) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cps_num_) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_pos_) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_vel_) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_acc_) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end_pos_) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'control_pts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'control_pts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'knots) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'knots)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendTraj>)))
  "Returns string type for a message object of type '<SendTraj>"
  "multi_bspline_opt/SendTraj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendTraj)))
  "Returns string type for a message object of type 'SendTraj"
  "multi_bspline_opt/SendTraj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendTraj>)))
  "Returns md5sum for a message object of type '<SendTraj>"
  "d1c07ca91141848b23c4fb9cb569a4f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendTraj)))
  "Returns md5sum for a message object of type 'SendTraj"
  "d1c07ca91141848b23c4fb9cb569a4f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendTraj>)))
  "Returns full string definition for message of type '<SendTraj>"
  (cl:format cl:nil "int32 drone_id~%int64 traj_id~%int32 order~%int32 cps_num_~%# int32 Dim_~%# int32 TrajSampleRate~%# float64 beta~%time start_time~%~%~%geometry_msgs/Point start_pos_~%geometry_msgs/Point start_vel_~%geometry_msgs/Point start_acc_~%geometry_msgs/Point end_pos_~%# float64 start_pos_x~%# float64 start_pos_y~%# float64 start_vel_x~%# float64 start_vel_y~%# float64 start_acc_x~%# float64 start_acc_y~%# float64 end_pos_x~%# float64 end_pos_y~%# float64 yaw_rate~%~%geometry_msgs/Point[] control_pts~%float64[] knots~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendTraj)))
  "Returns full string definition for message of type 'SendTraj"
  (cl:format cl:nil "int32 drone_id~%int64 traj_id~%int32 order~%int32 cps_num_~%# int32 Dim_~%# int32 TrajSampleRate~%# float64 beta~%time start_time~%~%~%geometry_msgs/Point start_pos_~%geometry_msgs/Point start_vel_~%geometry_msgs/Point start_acc_~%geometry_msgs/Point end_pos_~%# float64 start_pos_x~%# float64 start_pos_y~%# float64 start_vel_x~%# float64 start_vel_y~%# float64 start_acc_x~%# float64 start_acc_y~%# float64 end_pos_x~%# float64 end_pos_y~%# float64 yaw_rate~%~%geometry_msgs/Point[] control_pts~%float64[] knots~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendTraj>))
  (cl:+ 0
     4
     8
     4
     4
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_pos_))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_vel_))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_acc_))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end_pos_))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'control_pts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'knots) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendTraj>))
  "Converts a ROS message object to a list"
  (cl:list 'SendTraj
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':traj_id (traj_id msg))
    (cl:cons ':order (order msg))
    (cl:cons ':cps_num_ (cps_num_ msg))
    (cl:cons ':start_time (start_time msg))
    (cl:cons ':start_pos_ (start_pos_ msg))
    (cl:cons ':start_vel_ (start_vel_ msg))
    (cl:cons ':start_acc_ (start_acc_ msg))
    (cl:cons ':end_pos_ (end_pos_ msg))
    (cl:cons ':control_pts (control_pts msg))
    (cl:cons ':knots (knots msg))
))
