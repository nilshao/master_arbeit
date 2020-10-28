# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from franka_msgs/Errors.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Errors(genpy.Message):
  _md5sum = "420c43ed3349b66b110f24531e7b53e5"
  _type = "franka_msgs/Errors"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool joint_position_limits_violation
bool cartesian_position_limits_violation
bool self_collision_avoidance_violation
bool joint_velocity_violation
bool cartesian_velocity_violation
bool force_control_safety_violation
bool joint_reflex
bool cartesian_reflex
bool max_goal_pose_deviation_violation
bool max_path_pose_deviation_violation
bool cartesian_velocity_profile_safety_violation
bool joint_position_motion_generator_start_pose_invalid
bool joint_motion_generator_position_limits_violation
bool joint_motion_generator_velocity_limits_violation
bool joint_motion_generator_velocity_discontinuity
bool joint_motion_generator_acceleration_discontinuity
bool cartesian_position_motion_generator_start_pose_invalid
bool cartesian_motion_generator_elbow_limit_violation
bool cartesian_motion_generator_velocity_limits_violation
bool cartesian_motion_generator_velocity_discontinuity
bool cartesian_motion_generator_acceleration_discontinuity
bool cartesian_motion_generator_elbow_sign_inconsistent
bool cartesian_motion_generator_start_elbow_invalid
bool cartesian_motion_generator_joint_position_limits_violation
bool cartesian_motion_generator_joint_velocity_limits_violation
bool cartesian_motion_generator_joint_velocity_discontinuity
bool cartesian_motion_generator_joint_acceleration_discontinuity
bool cartesian_position_motion_generator_invalid_frame
bool force_controller_desired_force_tolerance_violation
bool controller_torque_discontinuity
bool start_elbow_sign_inconsistent
bool communication_constraints_violation
bool power_limit_violation
bool joint_p2p_insufficient_torque_for_planning
bool tau_j_range_violation
bool instability_detected
"""
  __slots__ = ['joint_position_limits_violation','cartesian_position_limits_violation','self_collision_avoidance_violation','joint_velocity_violation','cartesian_velocity_violation','force_control_safety_violation','joint_reflex','cartesian_reflex','max_goal_pose_deviation_violation','max_path_pose_deviation_violation','cartesian_velocity_profile_safety_violation','joint_position_motion_generator_start_pose_invalid','joint_motion_generator_position_limits_violation','joint_motion_generator_velocity_limits_violation','joint_motion_generator_velocity_discontinuity','joint_motion_generator_acceleration_discontinuity','cartesian_position_motion_generator_start_pose_invalid','cartesian_motion_generator_elbow_limit_violation','cartesian_motion_generator_velocity_limits_violation','cartesian_motion_generator_velocity_discontinuity','cartesian_motion_generator_acceleration_discontinuity','cartesian_motion_generator_elbow_sign_inconsistent','cartesian_motion_generator_start_elbow_invalid','cartesian_motion_generator_joint_position_limits_violation','cartesian_motion_generator_joint_velocity_limits_violation','cartesian_motion_generator_joint_velocity_discontinuity','cartesian_motion_generator_joint_acceleration_discontinuity','cartesian_position_motion_generator_invalid_frame','force_controller_desired_force_tolerance_violation','controller_torque_discontinuity','start_elbow_sign_inconsistent','communication_constraints_violation','power_limit_violation','joint_p2p_insufficient_torque_for_planning','tau_j_range_violation','instability_detected']
  _slot_types = ['bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       joint_position_limits_violation,cartesian_position_limits_violation,self_collision_avoidance_violation,joint_velocity_violation,cartesian_velocity_violation,force_control_safety_violation,joint_reflex,cartesian_reflex,max_goal_pose_deviation_violation,max_path_pose_deviation_violation,cartesian_velocity_profile_safety_violation,joint_position_motion_generator_start_pose_invalid,joint_motion_generator_position_limits_violation,joint_motion_generator_velocity_limits_violation,joint_motion_generator_velocity_discontinuity,joint_motion_generator_acceleration_discontinuity,cartesian_position_motion_generator_start_pose_invalid,cartesian_motion_generator_elbow_limit_violation,cartesian_motion_generator_velocity_limits_violation,cartesian_motion_generator_velocity_discontinuity,cartesian_motion_generator_acceleration_discontinuity,cartesian_motion_generator_elbow_sign_inconsistent,cartesian_motion_generator_start_elbow_invalid,cartesian_motion_generator_joint_position_limits_violation,cartesian_motion_generator_joint_velocity_limits_violation,cartesian_motion_generator_joint_velocity_discontinuity,cartesian_motion_generator_joint_acceleration_discontinuity,cartesian_position_motion_generator_invalid_frame,force_controller_desired_force_tolerance_violation,controller_torque_discontinuity,start_elbow_sign_inconsistent,communication_constraints_violation,power_limit_violation,joint_p2p_insufficient_torque_for_planning,tau_j_range_violation,instability_detected

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Errors, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.joint_position_limits_violation is None:
        self.joint_position_limits_violation = False
      if self.cartesian_position_limits_violation is None:
        self.cartesian_position_limits_violation = False
      if self.self_collision_avoidance_violation is None:
        self.self_collision_avoidance_violation = False
      if self.joint_velocity_violation is None:
        self.joint_velocity_violation = False
      if self.cartesian_velocity_violation is None:
        self.cartesian_velocity_violation = False
      if self.force_control_safety_violation is None:
        self.force_control_safety_violation = False
      if self.joint_reflex is None:
        self.joint_reflex = False
      if self.cartesian_reflex is None:
        self.cartesian_reflex = False
      if self.max_goal_pose_deviation_violation is None:
        self.max_goal_pose_deviation_violation = False
      if self.max_path_pose_deviation_violation is None:
        self.max_path_pose_deviation_violation = False
      if self.cartesian_velocity_profile_safety_violation is None:
        self.cartesian_velocity_profile_safety_violation = False
      if self.joint_position_motion_generator_start_pose_invalid is None:
        self.joint_position_motion_generator_start_pose_invalid = False
      if self.joint_motion_generator_position_limits_violation is None:
        self.joint_motion_generator_position_limits_violation = False
      if self.joint_motion_generator_velocity_limits_violation is None:
        self.joint_motion_generator_velocity_limits_violation = False
      if self.joint_motion_generator_velocity_discontinuity is None:
        self.joint_motion_generator_velocity_discontinuity = False
      if self.joint_motion_generator_acceleration_discontinuity is None:
        self.joint_motion_generator_acceleration_discontinuity = False
      if self.cartesian_position_motion_generator_start_pose_invalid is None:
        self.cartesian_position_motion_generator_start_pose_invalid = False
      if self.cartesian_motion_generator_elbow_limit_violation is None:
        self.cartesian_motion_generator_elbow_limit_violation = False
      if self.cartesian_motion_generator_velocity_limits_violation is None:
        self.cartesian_motion_generator_velocity_limits_violation = False
      if self.cartesian_motion_generator_velocity_discontinuity is None:
        self.cartesian_motion_generator_velocity_discontinuity = False
      if self.cartesian_motion_generator_acceleration_discontinuity is None:
        self.cartesian_motion_generator_acceleration_discontinuity = False
      if self.cartesian_motion_generator_elbow_sign_inconsistent is None:
        self.cartesian_motion_generator_elbow_sign_inconsistent = False
      if self.cartesian_motion_generator_start_elbow_invalid is None:
        self.cartesian_motion_generator_start_elbow_invalid = False
      if self.cartesian_motion_generator_joint_position_limits_violation is None:
        self.cartesian_motion_generator_joint_position_limits_violation = False
      if self.cartesian_motion_generator_joint_velocity_limits_violation is None:
        self.cartesian_motion_generator_joint_velocity_limits_violation = False
      if self.cartesian_motion_generator_joint_velocity_discontinuity is None:
        self.cartesian_motion_generator_joint_velocity_discontinuity = False
      if self.cartesian_motion_generator_joint_acceleration_discontinuity is None:
        self.cartesian_motion_generator_joint_acceleration_discontinuity = False
      if self.cartesian_position_motion_generator_invalid_frame is None:
        self.cartesian_position_motion_generator_invalid_frame = False
      if self.force_controller_desired_force_tolerance_violation is None:
        self.force_controller_desired_force_tolerance_violation = False
      if self.controller_torque_discontinuity is None:
        self.controller_torque_discontinuity = False
      if self.start_elbow_sign_inconsistent is None:
        self.start_elbow_sign_inconsistent = False
      if self.communication_constraints_violation is None:
        self.communication_constraints_violation = False
      if self.power_limit_violation is None:
        self.power_limit_violation = False
      if self.joint_p2p_insufficient_torque_for_planning is None:
        self.joint_p2p_insufficient_torque_for_planning = False
      if self.tau_j_range_violation is None:
        self.tau_j_range_violation = False
      if self.instability_detected is None:
        self.instability_detected = False
    else:
      self.joint_position_limits_violation = False
      self.cartesian_position_limits_violation = False
      self.self_collision_avoidance_violation = False
      self.joint_velocity_violation = False
      self.cartesian_velocity_violation = False
      self.force_control_safety_violation = False
      self.joint_reflex = False
      self.cartesian_reflex = False
      self.max_goal_pose_deviation_violation = False
      self.max_path_pose_deviation_violation = False
      self.cartesian_velocity_profile_safety_violation = False
      self.joint_position_motion_generator_start_pose_invalid = False
      self.joint_motion_generator_position_limits_violation = False
      self.joint_motion_generator_velocity_limits_violation = False
      self.joint_motion_generator_velocity_discontinuity = False
      self.joint_motion_generator_acceleration_discontinuity = False
      self.cartesian_position_motion_generator_start_pose_invalid = False
      self.cartesian_motion_generator_elbow_limit_violation = False
      self.cartesian_motion_generator_velocity_limits_violation = False
      self.cartesian_motion_generator_velocity_discontinuity = False
      self.cartesian_motion_generator_acceleration_discontinuity = False
      self.cartesian_motion_generator_elbow_sign_inconsistent = False
      self.cartesian_motion_generator_start_elbow_invalid = False
      self.cartesian_motion_generator_joint_position_limits_violation = False
      self.cartesian_motion_generator_joint_velocity_limits_violation = False
      self.cartesian_motion_generator_joint_velocity_discontinuity = False
      self.cartesian_motion_generator_joint_acceleration_discontinuity = False
      self.cartesian_position_motion_generator_invalid_frame = False
      self.force_controller_desired_force_tolerance_violation = False
      self.controller_torque_discontinuity = False
      self.start_elbow_sign_inconsistent = False
      self.communication_constraints_violation = False
      self.power_limit_violation = False
      self.joint_p2p_insufficient_torque_for_planning = False
      self.tau_j_range_violation = False
      self.instability_detected = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_36B().pack(_x.joint_position_limits_violation, _x.cartesian_position_limits_violation, _x.self_collision_avoidance_violation, _x.joint_velocity_violation, _x.cartesian_velocity_violation, _x.force_control_safety_violation, _x.joint_reflex, _x.cartesian_reflex, _x.max_goal_pose_deviation_violation, _x.max_path_pose_deviation_violation, _x.cartesian_velocity_profile_safety_violation, _x.joint_position_motion_generator_start_pose_invalid, _x.joint_motion_generator_position_limits_violation, _x.joint_motion_generator_velocity_limits_violation, _x.joint_motion_generator_velocity_discontinuity, _x.joint_motion_generator_acceleration_discontinuity, _x.cartesian_position_motion_generator_start_pose_invalid, _x.cartesian_motion_generator_elbow_limit_violation, _x.cartesian_motion_generator_velocity_limits_violation, _x.cartesian_motion_generator_velocity_discontinuity, _x.cartesian_motion_generator_acceleration_discontinuity, _x.cartesian_motion_generator_elbow_sign_inconsistent, _x.cartesian_motion_generator_start_elbow_invalid, _x.cartesian_motion_generator_joint_position_limits_violation, _x.cartesian_motion_generator_joint_velocity_limits_violation, _x.cartesian_motion_generator_joint_velocity_discontinuity, _x.cartesian_motion_generator_joint_acceleration_discontinuity, _x.cartesian_position_motion_generator_invalid_frame, _x.force_controller_desired_force_tolerance_violation, _x.controller_torque_discontinuity, _x.start_elbow_sign_inconsistent, _x.communication_constraints_violation, _x.power_limit_violation, _x.joint_p2p_insufficient_torque_for_planning, _x.tau_j_range_violation, _x.instability_detected))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 36
      (_x.joint_position_limits_violation, _x.cartesian_position_limits_violation, _x.self_collision_avoidance_violation, _x.joint_velocity_violation, _x.cartesian_velocity_violation, _x.force_control_safety_violation, _x.joint_reflex, _x.cartesian_reflex, _x.max_goal_pose_deviation_violation, _x.max_path_pose_deviation_violation, _x.cartesian_velocity_profile_safety_violation, _x.joint_position_motion_generator_start_pose_invalid, _x.joint_motion_generator_position_limits_violation, _x.joint_motion_generator_velocity_limits_violation, _x.joint_motion_generator_velocity_discontinuity, _x.joint_motion_generator_acceleration_discontinuity, _x.cartesian_position_motion_generator_start_pose_invalid, _x.cartesian_motion_generator_elbow_limit_violation, _x.cartesian_motion_generator_velocity_limits_violation, _x.cartesian_motion_generator_velocity_discontinuity, _x.cartesian_motion_generator_acceleration_discontinuity, _x.cartesian_motion_generator_elbow_sign_inconsistent, _x.cartesian_motion_generator_start_elbow_invalid, _x.cartesian_motion_generator_joint_position_limits_violation, _x.cartesian_motion_generator_joint_velocity_limits_violation, _x.cartesian_motion_generator_joint_velocity_discontinuity, _x.cartesian_motion_generator_joint_acceleration_discontinuity, _x.cartesian_position_motion_generator_invalid_frame, _x.force_controller_desired_force_tolerance_violation, _x.controller_torque_discontinuity, _x.start_elbow_sign_inconsistent, _x.communication_constraints_violation, _x.power_limit_violation, _x.joint_p2p_insufficient_torque_for_planning, _x.tau_j_range_violation, _x.instability_detected,) = _get_struct_36B().unpack(str[start:end])
      self.joint_position_limits_violation = bool(self.joint_position_limits_violation)
      self.cartesian_position_limits_violation = bool(self.cartesian_position_limits_violation)
      self.self_collision_avoidance_violation = bool(self.self_collision_avoidance_violation)
      self.joint_velocity_violation = bool(self.joint_velocity_violation)
      self.cartesian_velocity_violation = bool(self.cartesian_velocity_violation)
      self.force_control_safety_violation = bool(self.force_control_safety_violation)
      self.joint_reflex = bool(self.joint_reflex)
      self.cartesian_reflex = bool(self.cartesian_reflex)
      self.max_goal_pose_deviation_violation = bool(self.max_goal_pose_deviation_violation)
      self.max_path_pose_deviation_violation = bool(self.max_path_pose_deviation_violation)
      self.cartesian_velocity_profile_safety_violation = bool(self.cartesian_velocity_profile_safety_violation)
      self.joint_position_motion_generator_start_pose_invalid = bool(self.joint_position_motion_generator_start_pose_invalid)
      self.joint_motion_generator_position_limits_violation = bool(self.joint_motion_generator_position_limits_violation)
      self.joint_motion_generator_velocity_limits_violation = bool(self.joint_motion_generator_velocity_limits_violation)
      self.joint_motion_generator_velocity_discontinuity = bool(self.joint_motion_generator_velocity_discontinuity)
      self.joint_motion_generator_acceleration_discontinuity = bool(self.joint_motion_generator_acceleration_discontinuity)
      self.cartesian_position_motion_generator_start_pose_invalid = bool(self.cartesian_position_motion_generator_start_pose_invalid)
      self.cartesian_motion_generator_elbow_limit_violation = bool(self.cartesian_motion_generator_elbow_limit_violation)
      self.cartesian_motion_generator_velocity_limits_violation = bool(self.cartesian_motion_generator_velocity_limits_violation)
      self.cartesian_motion_generator_velocity_discontinuity = bool(self.cartesian_motion_generator_velocity_discontinuity)
      self.cartesian_motion_generator_acceleration_discontinuity = bool(self.cartesian_motion_generator_acceleration_discontinuity)
      self.cartesian_motion_generator_elbow_sign_inconsistent = bool(self.cartesian_motion_generator_elbow_sign_inconsistent)
      self.cartesian_motion_generator_start_elbow_invalid = bool(self.cartesian_motion_generator_start_elbow_invalid)
      self.cartesian_motion_generator_joint_position_limits_violation = bool(self.cartesian_motion_generator_joint_position_limits_violation)
      self.cartesian_motion_generator_joint_velocity_limits_violation = bool(self.cartesian_motion_generator_joint_velocity_limits_violation)
      self.cartesian_motion_generator_joint_velocity_discontinuity = bool(self.cartesian_motion_generator_joint_velocity_discontinuity)
      self.cartesian_motion_generator_joint_acceleration_discontinuity = bool(self.cartesian_motion_generator_joint_acceleration_discontinuity)
      self.cartesian_position_motion_generator_invalid_frame = bool(self.cartesian_position_motion_generator_invalid_frame)
      self.force_controller_desired_force_tolerance_violation = bool(self.force_controller_desired_force_tolerance_violation)
      self.controller_torque_discontinuity = bool(self.controller_torque_discontinuity)
      self.start_elbow_sign_inconsistent = bool(self.start_elbow_sign_inconsistent)
      self.communication_constraints_violation = bool(self.communication_constraints_violation)
      self.power_limit_violation = bool(self.power_limit_violation)
      self.joint_p2p_insufficient_torque_for_planning = bool(self.joint_p2p_insufficient_torque_for_planning)
      self.tau_j_range_violation = bool(self.tau_j_range_violation)
      self.instability_detected = bool(self.instability_detected)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_36B().pack(_x.joint_position_limits_violation, _x.cartesian_position_limits_violation, _x.self_collision_avoidance_violation, _x.joint_velocity_violation, _x.cartesian_velocity_violation, _x.force_control_safety_violation, _x.joint_reflex, _x.cartesian_reflex, _x.max_goal_pose_deviation_violation, _x.max_path_pose_deviation_violation, _x.cartesian_velocity_profile_safety_violation, _x.joint_position_motion_generator_start_pose_invalid, _x.joint_motion_generator_position_limits_violation, _x.joint_motion_generator_velocity_limits_violation, _x.joint_motion_generator_velocity_discontinuity, _x.joint_motion_generator_acceleration_discontinuity, _x.cartesian_position_motion_generator_start_pose_invalid, _x.cartesian_motion_generator_elbow_limit_violation, _x.cartesian_motion_generator_velocity_limits_violation, _x.cartesian_motion_generator_velocity_discontinuity, _x.cartesian_motion_generator_acceleration_discontinuity, _x.cartesian_motion_generator_elbow_sign_inconsistent, _x.cartesian_motion_generator_start_elbow_invalid, _x.cartesian_motion_generator_joint_position_limits_violation, _x.cartesian_motion_generator_joint_velocity_limits_violation, _x.cartesian_motion_generator_joint_velocity_discontinuity, _x.cartesian_motion_generator_joint_acceleration_discontinuity, _x.cartesian_position_motion_generator_invalid_frame, _x.force_controller_desired_force_tolerance_violation, _x.controller_torque_discontinuity, _x.start_elbow_sign_inconsistent, _x.communication_constraints_violation, _x.power_limit_violation, _x.joint_p2p_insufficient_torque_for_planning, _x.tau_j_range_violation, _x.instability_detected))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 36
      (_x.joint_position_limits_violation, _x.cartesian_position_limits_violation, _x.self_collision_avoidance_violation, _x.joint_velocity_violation, _x.cartesian_velocity_violation, _x.force_control_safety_violation, _x.joint_reflex, _x.cartesian_reflex, _x.max_goal_pose_deviation_violation, _x.max_path_pose_deviation_violation, _x.cartesian_velocity_profile_safety_violation, _x.joint_position_motion_generator_start_pose_invalid, _x.joint_motion_generator_position_limits_violation, _x.joint_motion_generator_velocity_limits_violation, _x.joint_motion_generator_velocity_discontinuity, _x.joint_motion_generator_acceleration_discontinuity, _x.cartesian_position_motion_generator_start_pose_invalid, _x.cartesian_motion_generator_elbow_limit_violation, _x.cartesian_motion_generator_velocity_limits_violation, _x.cartesian_motion_generator_velocity_discontinuity, _x.cartesian_motion_generator_acceleration_discontinuity, _x.cartesian_motion_generator_elbow_sign_inconsistent, _x.cartesian_motion_generator_start_elbow_invalid, _x.cartesian_motion_generator_joint_position_limits_violation, _x.cartesian_motion_generator_joint_velocity_limits_violation, _x.cartesian_motion_generator_joint_velocity_discontinuity, _x.cartesian_motion_generator_joint_acceleration_discontinuity, _x.cartesian_position_motion_generator_invalid_frame, _x.force_controller_desired_force_tolerance_violation, _x.controller_torque_discontinuity, _x.start_elbow_sign_inconsistent, _x.communication_constraints_violation, _x.power_limit_violation, _x.joint_p2p_insufficient_torque_for_planning, _x.tau_j_range_violation, _x.instability_detected,) = _get_struct_36B().unpack(str[start:end])
      self.joint_position_limits_violation = bool(self.joint_position_limits_violation)
      self.cartesian_position_limits_violation = bool(self.cartesian_position_limits_violation)
      self.self_collision_avoidance_violation = bool(self.self_collision_avoidance_violation)
      self.joint_velocity_violation = bool(self.joint_velocity_violation)
      self.cartesian_velocity_violation = bool(self.cartesian_velocity_violation)
      self.force_control_safety_violation = bool(self.force_control_safety_violation)
      self.joint_reflex = bool(self.joint_reflex)
      self.cartesian_reflex = bool(self.cartesian_reflex)
      self.max_goal_pose_deviation_violation = bool(self.max_goal_pose_deviation_violation)
      self.max_path_pose_deviation_violation = bool(self.max_path_pose_deviation_violation)
      self.cartesian_velocity_profile_safety_violation = bool(self.cartesian_velocity_profile_safety_violation)
      self.joint_position_motion_generator_start_pose_invalid = bool(self.joint_position_motion_generator_start_pose_invalid)
      self.joint_motion_generator_position_limits_violation = bool(self.joint_motion_generator_position_limits_violation)
      self.joint_motion_generator_velocity_limits_violation = bool(self.joint_motion_generator_velocity_limits_violation)
      self.joint_motion_generator_velocity_discontinuity = bool(self.joint_motion_generator_velocity_discontinuity)
      self.joint_motion_generator_acceleration_discontinuity = bool(self.joint_motion_generator_acceleration_discontinuity)
      self.cartesian_position_motion_generator_start_pose_invalid = bool(self.cartesian_position_motion_generator_start_pose_invalid)
      self.cartesian_motion_generator_elbow_limit_violation = bool(self.cartesian_motion_generator_elbow_limit_violation)
      self.cartesian_motion_generator_velocity_limits_violation = bool(self.cartesian_motion_generator_velocity_limits_violation)
      self.cartesian_motion_generator_velocity_discontinuity = bool(self.cartesian_motion_generator_velocity_discontinuity)
      self.cartesian_motion_generator_acceleration_discontinuity = bool(self.cartesian_motion_generator_acceleration_discontinuity)
      self.cartesian_motion_generator_elbow_sign_inconsistent = bool(self.cartesian_motion_generator_elbow_sign_inconsistent)
      self.cartesian_motion_generator_start_elbow_invalid = bool(self.cartesian_motion_generator_start_elbow_invalid)
      self.cartesian_motion_generator_joint_position_limits_violation = bool(self.cartesian_motion_generator_joint_position_limits_violation)
      self.cartesian_motion_generator_joint_velocity_limits_violation = bool(self.cartesian_motion_generator_joint_velocity_limits_violation)
      self.cartesian_motion_generator_joint_velocity_discontinuity = bool(self.cartesian_motion_generator_joint_velocity_discontinuity)
      self.cartesian_motion_generator_joint_acceleration_discontinuity = bool(self.cartesian_motion_generator_joint_acceleration_discontinuity)
      self.cartesian_position_motion_generator_invalid_frame = bool(self.cartesian_position_motion_generator_invalid_frame)
      self.force_controller_desired_force_tolerance_violation = bool(self.force_controller_desired_force_tolerance_violation)
      self.controller_torque_discontinuity = bool(self.controller_torque_discontinuity)
      self.start_elbow_sign_inconsistent = bool(self.start_elbow_sign_inconsistent)
      self.communication_constraints_violation = bool(self.communication_constraints_violation)
      self.power_limit_violation = bool(self.power_limit_violation)
      self.joint_p2p_insufficient_torque_for_planning = bool(self.joint_p2p_insufficient_torque_for_planning)
      self.tau_j_range_violation = bool(self.tau_j_range_violation)
      self.instability_detected = bool(self.instability_detected)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_36B = None
def _get_struct_36B():
    global _struct_36B
    if _struct_36B is None:
        _struct_36B = struct.Struct("<36B")
    return _struct_36B
