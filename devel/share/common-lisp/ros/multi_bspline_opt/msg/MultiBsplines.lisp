; Auto-generated. Do not edit!


(cl:in-package multi_bspline_opt-msg)


;//! \htmlinclude MultiBsplines.msg.html

(cl:defclass <MultiBsplines> (roslisp-msg-protocol:ros-message)
  ((drone_id_from
    :reader drone_id_from
    :initarg :drone_id_from
    :type cl:integer
    :initform 0)
   (traj
    :reader traj
    :initarg :traj
    :type (cl:vector multi_bspline_opt-msg:SendTraj)
   :initform (cl:make-array 0 :element-type 'multi_bspline_opt-msg:SendTraj :initial-element (cl:make-instance 'multi_bspline_opt-msg:SendTraj))))
)

(cl:defclass MultiBsplines (<MultiBsplines>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MultiBsplines>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MultiBsplines)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name multi_bspline_opt-msg:<MultiBsplines> is deprecated: use multi_bspline_opt-msg:MultiBsplines instead.")))

(cl:ensure-generic-function 'drone_id_from-val :lambda-list '(m))
(cl:defmethod drone_id_from-val ((m <MultiBsplines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:drone_id_from-val is deprecated.  Use multi_bspline_opt-msg:drone_id_from instead.")
  (drone_id_from m))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <MultiBsplines>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader multi_bspline_opt-msg:traj-val is deprecated.  Use multi_bspline_opt-msg:traj instead.")
  (traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MultiBsplines>) ostream)
  "Serializes a message object of type '<MultiBsplines>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id_from)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'traj))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'traj))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MultiBsplines>) istream)
  "Deserializes a message object of type '<MultiBsplines>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id_from) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'traj) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'traj)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'multi_bspline_opt-msg:SendTraj))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MultiBsplines>)))
  "Returns string type for a message object of type '<MultiBsplines>"
  "multi_bspline_opt/MultiBsplines")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MultiBsplines)))
  "Returns string type for a message object of type 'MultiBsplines"
  "multi_bspline_opt/MultiBsplines")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MultiBsplines>)))
  "Returns md5sum for a message object of type '<MultiBsplines>"
<<<<<<< HEAD
  "cdca54030719384ffd1c921d966beb6e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiBsplines)))
  "Returns md5sum for a message object of type 'MultiBsplines"
  "cdca54030719384ffd1c921d966beb6e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiBsplines>)))
  "Returns full string definition for message of type '<MultiBsplines>"
  (cl:format cl:nil "int32 drone_id_from~%~%SendTraj[] traj~%================================================================================~%MSG: multi_bspline_opt/SendTraj~%int32 drone_id~%int64 traj_id~%int32 order~%int32 cps_num_~%# int32 Dim_~%# int32 TrajSampleRate~%# float64 beta~%time start_time~%~%~%float64 start_pos_x~%float64 start_pos_y~%float64 start_vel_x~%float64 start_vel_y~%float64 start_acc_x~%float64 start_acc_y~%float64 end_pos_x~%float64 end_pos_y~%# float64 yaw_rate~%~%geometry_msgs/Point[] control_pts~%float64[] knots~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiBsplines)))
  "Returns full string definition for message of type 'MultiBsplines"
  (cl:format cl:nil "int32 drone_id_from~%~%SendTraj[] traj~%================================================================================~%MSG: multi_bspline_opt/SendTraj~%int32 drone_id~%int64 traj_id~%int32 order~%int32 cps_num_~%# int32 Dim_~%# int32 TrajSampleRate~%# float64 beta~%time start_time~%~%~%float64 start_pos_x~%float64 start_pos_y~%float64 start_vel_x~%float64 start_vel_y~%float64 start_acc_x~%float64 start_acc_y~%float64 end_pos_x~%float64 end_pos_y~%# float64 yaw_rate~%~%geometry_msgs/Point[] control_pts~%float64[] knots~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
=======
  "0a9ce0a6a663879c244f8d58ac09d0d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiBsplines)))
  "Returns md5sum for a message object of type 'MultiBsplines"
  "0a9ce0a6a663879c244f8d58ac09d0d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiBsplines>)))
  "Returns full string definition for message of type '<MultiBsplines>"
  (cl:format cl:nil "int32 drone_id_from~%~%SendTraj[] traj~%================================================================================~%MSG: multi_bspline_opt/SendTraj~%int32 drone_id~%int64 traj_id~%int32 order~%int32 cps_num_~%# int32 Dim_~%# int32 TrajSampleRate~%# float64 beta~%time start_time~%~%~%float64 start_pos_x~%float64 start_pos_y~%float64 start_vel_x~%float64 start_vel_y~%float64 end_pos_x~%float64 end_pos_y~%# float64 yaw_rate~%~%geometry_msgs/Point[] control_pts~%float64[] knots~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiBsplines)))
  "Returns full string definition for message of type 'MultiBsplines"
  (cl:format cl:nil "int32 drone_id_from~%~%SendTraj[] traj~%================================================================================~%MSG: multi_bspline_opt/SendTraj~%int32 drone_id~%int64 traj_id~%int32 order~%int32 cps_num_~%# int32 Dim_~%# int32 TrajSampleRate~%# float64 beta~%time start_time~%~%~%float64 start_pos_x~%float64 start_pos_y~%float64 start_vel_x~%float64 start_vel_y~%float64 end_pos_x~%float64 end_pos_y~%# float64 yaw_rate~%~%geometry_msgs/Point[] control_pts~%float64[] knots~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MultiBsplines>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'traj) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MultiBsplines>))
  "Converts a ROS message object to a list"
  (cl:list 'MultiBsplines
    (cl:cons ':drone_id_from (drone_id_from msg))
    (cl:cons ':traj (traj msg))
))
