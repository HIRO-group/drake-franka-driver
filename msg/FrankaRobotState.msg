# Complete Franka Robot State Message
# Matches franka::RobotState structure
# https://docs.ros.org/en/kinetic/api/libfranka/html/structfranka_1_1RobotState.html

std_msgs/Header header

# =============================================================================
# JOINT STATES
# =============================================================================

# Measured joint position [rad]
float64[7] q

# Desired joint position [rad]
float64[7] q_d

# Measured joint velocity [rad/s]
float64[7] dq

# Desired joint velocity [rad/s]
float64[7] dq_d

# Desired joint acceleration [rad/s²]
float64[7] ddq_d

# Measured link-side joint torque sensor signals [Nm]
float64[7] tau_j

# Desired link-side joint torque sensor signals without gravity [Nm]
float64[7] tau_j_d

# Derivative of measured link-side joint torque sensor signals [Nm/s]
float64[7] dtau_j

# External torque, filtered [Nm]
float64[7] tau_ext_hat_filtered

# =============================================================================
# CARTESIAN POSES (4x4 matrices in column-major format)
# =============================================================================

# Measured end effector pose in base frame (4x4 matrix as 16 elements)
float64[16] o_t_ee

# Last desired end effector pose of motion generation in base frame
float64[16] o_t_ee_d

# Last commanded end effector pose of motion generation in base frame
float64[16] o_t_ee_c

# End effector frame pose in flange frame
float64[16] f_t_ee

# Stiffness frame pose in end effector frame
float64[16] ee_t_k

# =============================================================================
# CARTESIAN VELOCITIES & ACCELERATIONS
# =============================================================================

# Desired end effector twist in base frame [m/s, m/s, m/s, rad/s, rad/s, rad/s]
float64[6] o_dp_ee_d

# Last commanded end effector twist in base frame
float64[6] o_dp_ee_c

# Last commanded end effector acceleration in base frame [m/s², m/s², m/s², rad/s², rad/s², rad/s²]
float64[6] o_ddp_ee_c

# =============================================================================
# EXTERNAL FORCES & TORQUES
# =============================================================================

# Estimated external wrench (force, torque) acting on stiffness frame, 
# expressed relative to the base frame [N, N, N, Nm, Nm, Nm]
float64[6] o_f_ext_hat_k

# Estimated external wrench (force, torque) acting on stiffness frame,
# expressed relative to the stiffness frame [N, N, N, Nm, Nm, Nm]
float64[6] k_f_ext_hat_k

# =============================================================================
# ELBOW CONFIGURATION
# =============================================================================

# Elbow configuration: [position of 3rd joint (rad), sign of 4th joint (+1 or -1)]
float64[2] elbow

# Desired elbow configuration
float64[2] elbow_d

# Commanded elbow configuration
float64[2] elbow_c

# Commanded elbow velocity
float64[2] delbow_c

# Commanded elbow acceleration
float64[2] ddelbow_c

# =============================================================================
# MASS & INERTIA PROPERTIES
# =============================================================================

# Configured mass of the end effector [kg]
float64 m_ee

# Configured mass of the external load [kg]
float64 m_load

# Sum of the mass of the end effector and the external load [kg]
float64 m_total

# Configured center of mass of the end effector load with respect to flange frame [m]
float64[3] f_x_cee

# Configured center of mass of the external load with respect to flange frame [m]
float64[3] f_x_cload

# Combined center of mass of the end effector load and external load [m]
float64[3] f_x_ctotal

# Configured rotational inertia matrix of the end effector load (3x3 as 9 elements) [kg*m²]
float64[9] i_ee

# Configured rotational inertia matrix of the external load (3x3 as 9 elements) [kg*m²]
float64[9] i_load

# Combined rotational inertia matrix (3x3 as 9 elements) [kg*m²]
float64[9] i_total

# =============================================================================
# CONTACT & COLLISION DETECTION
# =============================================================================

# Indicates which contact level is activated in which joint
# After contact disappears, value turns to zero
float64[7] joint_contact

# Indicates which contact level is activated in which joint
# After contact disappears, the value stays the same until a reset command is sent
float64[7] joint_collision

# Indicates which contact level is activated in which Cartesian dimension (x,y,z,R,P,Y)
# After contact disappears, value turns to zero
float64[6] cartesian_contact

# Indicates which contact level is activated in which Cartesian dimension (x,y,z,R,P,Y)
# After contact disappears, the value stays the same until a reset command is sent
float64[6] cartesian_collision

# =============================================================================
# MOTOR STATES
# =============================================================================

# Motor position [rad]
float64[7] theta

# Motor velocity [rad/s]
float64[7] dtheta

# =============================================================================
# CONTROL STATUS
# =============================================================================

# Percentage of the last 100 control commands that were successfully received by the robot
float64 control_command_success_rate

# Current robot mode
uint8 robot_mode

# Robot mode constants
uint8 ROBOT_MODE_OTHER = 0
uint8 ROBOT_MODE_IDLE = 1
uint8 ROBOT_MODE_MOVE = 2
uint8 ROBOT_MODE_GUIDING = 3
uint8 ROBOT_MODE_REFLEX = 4
uint8 ROBOT_MODE_USER_STOPPED = 5
uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY = 6

# =============================================================================
# TIMING
# =============================================================================

# Strictly monotonically increasing timestamp since robot start [microseconds]
uint64 time_robot

