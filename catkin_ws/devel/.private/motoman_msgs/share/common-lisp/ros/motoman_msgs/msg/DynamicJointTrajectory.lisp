; Auto-generated. Do not edit!


(cl:in-package motoman_msgs-msg)


;//! \htmlinclude DynamicJointTrajectory.msg.html

(cl:defclass <DynamicJointTrajectory> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (joint_names
    :reader joint_names
    :initarg :joint_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (points
    :reader points
    :initarg :points
    :type (cl:vector motoman_msgs-msg:DynamicJointPoint)
   :initform (cl:make-array 0 :element-type 'motoman_msgs-msg:DynamicJointPoint :initial-element (cl:make-instance 'motoman_msgs-msg:DynamicJointPoint))))
)

(cl:defclass DynamicJointTrajectory (<DynamicJointTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DynamicJointTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DynamicJointTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motoman_msgs-msg:<DynamicJointTrajectory> is deprecated: use motoman_msgs-msg:DynamicJointTrajectory instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DynamicJointTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-msg:header-val is deprecated.  Use motoman_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'joint_names-val :lambda-list '(m))
(cl:defmethod joint_names-val ((m <DynamicJointTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-msg:joint_names-val is deprecated.  Use motoman_msgs-msg:joint_names instead.")
  (joint_names m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <DynamicJointTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motoman_msgs-msg:points-val is deprecated.  Use motoman_msgs-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DynamicJointTrajectory>) ostream)
  "Serializes a message object of type '<DynamicJointTrajectory>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'joint_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DynamicJointTrajectory>) istream)
  "Deserializes a message object of type '<DynamicJointTrajectory>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'motoman_msgs-msg:DynamicJointPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DynamicJointTrajectory>)))
  "Returns string type for a message object of type '<DynamicJointTrajectory>"
  "motoman_msgs/DynamicJointTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DynamicJointTrajectory)))
  "Returns string type for a message object of type 'DynamicJointTrajectory"
  "motoman_msgs/DynamicJointTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DynamicJointTrajectory>)))
  "Returns md5sum for a message object of type '<DynamicJointTrajectory>"
  "81bfbf2d02070fdef3a528bd72b49257")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DynamicJointTrajectory)))
  "Returns md5sum for a message object of type 'DynamicJointTrajectory"
  "81bfbf2d02070fdef3a528bd72b49257")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DynamicJointTrajectory>)))
  "Returns full string definition for message of type '<DynamicJointTrajectory>"
  (cl:format cl:nil "#length: true message/data length~%#header: ~%#sequence:~%#num_groups: # of motion groups included in this message~%#group[]: DynamicJointPoint from DynamicJointPoint.msg~%~%Header header~%string[] joint_names~%DynamicJointPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motoman_msgs/DynamicJointPoint~%# DynamicJointPoint~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%int16 num_groups~%DynamicJointsGroup[] groups~%~%================================================================================~%MSG: motoman_msgs/DynamicJointsGroup~%# DynamicJointsGroup~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%~%int16 group_number~%int16 num_joints~%int16 valid_fields~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DynamicJointTrajectory)))
  "Returns full string definition for message of type 'DynamicJointTrajectory"
  (cl:format cl:nil "#length: true message/data length~%#header: ~%#sequence:~%#num_groups: # of motion groups included in this message~%#group[]: DynamicJointPoint from DynamicJointPoint.msg~%~%Header header~%string[] joint_names~%DynamicJointPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: motoman_msgs/DynamicJointPoint~%# DynamicJointPoint~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%int16 num_groups~%DynamicJointsGroup[] groups~%~%================================================================================~%MSG: motoman_msgs/DynamicJointsGroup~%# DynamicJointsGroup~%#group: # length of this array must match num_groups~%#    id:   control-group ID for use on-controller~%#    num_joints: # of joints in this motion group~%#    valid_fields: #bit field for following items~%#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.~%#    positions[]~%#    velocities[]~%#    accelerations[]~%#    effort[]~%#    time_from_start~%~%~%int16 group_number~%int16 num_joints~%int16 valid_fields~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DynamicJointTrajectory>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DynamicJointTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'DynamicJointTrajectory
    (cl:cons ':header (header msg))
    (cl:cons ':joint_names (joint_names msg))
    (cl:cons ':points (points msg))
))
