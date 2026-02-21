from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class TaksT1Cfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.45] # x,y,z [m]
        rot = [0.0, 0, 0, 1.0] # x,y,z,w [quat]
        target_joint_angles = { # = target angles [rad] when action = 0.0
           'left_hip_pitch_joint' : -0.14,
           'left_hip_roll_joint' : 0,
           'left_hip_yaw_joint' : 0.,
           'left_knee_joint' : 0.36,
           'left_ankle_pitch_joint' : -0.20,
           'left_ankle_roll_joint' : 0,
           'right_hip_pitch_joint' : -0.14,
           'right_hip_roll_joint' : 0,
           'right_hip_yaw_joint' : 0.,
           'right_knee_joint' : 0.36,
           'right_ankle_pitch_joint': -0.20,
           'right_ankle_roll_joint' : 0,
           'waist_yaw_joint' : 0.0,
           'waist_roll_joint' : 0.0,
           'waist_pitch_joint' : 0.0,
           'left_shoulder_pitch_joint' : 0.16,
           'left_shoulder_roll_joint' : 0.16,
           'left_shoulder_yaw_joint' : 0.0,
           'left_elbow_joint' : 1.10,
           'left_wrist_roll_joint' : 0,
           'left_wrist_yaw_joint' : 0,
           'left_wrist_pitch_joint' : 0,
           'right_shoulder_pitch_joint' : 0.16,
           'right_shoulder_roll_joint' : -0.16,
           'right_shoulder_yaw_joint' : 0.0,
           'right_elbow_joint' : 1.10,
           'right_wrist_roll_joint' : 0,
           'right_wrist_yaw_joint' : 0,
           'right_wrist_pitch_joint' : 0,
           'neck_yaw_joint' : 0,
           'neck_roll_joint' : 0,
           'neck_pitch_joint' : 0,
        }
        default_joint_angles = { # = target angles [rad] when action = 0.0
           'left_hip_pitch_joint' : -1.5,
           'left_hip_roll_joint' : 0,
           'left_hip_yaw_joint' : 0.,
           'left_knee_joint' : 0,
           'left_ankle_pitch_joint' : -0.20,
           'left_ankle_roll_joint' : 0,
           'right_hip_pitch_joint' : -1.5,
           'right_hip_roll_joint' : 0,
           'right_hip_yaw_joint' : 0.,
           'right_knee_joint' : 0,
           'right_ankle_pitch_joint': -0.20,
           'right_ankle_roll_joint' : 0,
           'waist_yaw_joint' : 0.0,
           'waist_roll_joint' : 0.0,
           'waist_pitch_joint' : 0.0,
           'left_shoulder_pitch_joint' : 0.,
           'left_shoulder_roll_joint' : 0.,
           'left_shoulder_yaw_joint' : 0.0,
           'left_elbow_joint' : 1.10,
           'left_wrist_roll_joint' : 0,
           'left_wrist_yaw_joint' : 0,
           'left_wrist_pitch_joint' : 0,
           'right_shoulder_pitch_joint' : 0.,
           'right_shoulder_roll_joint' : 0.,
           'right_shoulder_yaw_joint' : 0.0,
           'right_elbow_joint' : 1.10,
           'right_wrist_roll_joint' : 0,
           'right_wrist_yaw_joint' : 0,
           'right_wrist_pitch_joint' : 0,
           'neck_yaw_joint' : 0,
           'neck_roll_joint' : 0,
           'neck_pitch_joint' : 0,
        }

    class env(LeggedRobotCfg.env):
        num_one_step_observations= 103
        num_actions = 32
        num_dofs = 32
        num_actor_history = 6
        num_observations = num_actor_history * num_one_step_observations
        episode_length_s = 10 # episode length in seconds
        unactuated_timesteps = 50

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'hip': 150,
                     'knee': 200,
                     'ankle': 40,
                     'shoulder': 100,
                     'elbow': 100,
                     'waist': 150,
                     'wrist': 20,
                     'neck': 10,
                     }  # [N*m/rad]
        damping = {  'hip': 3.3,
                     'knee': 5,
                     'ankle': 2,
                     'shoulder': 2,
                     'elbow': 2,
                     'waist': 5,
                     'wrist': 1,
                     'neck': 0.25,
                     }  # [N*m*s/rad]
        action_scale = 1
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class terrain:
        mesh_type = 'trimesh' # "heightfield" # none, plane, heightfield or trimesh
        horizontal_scale = 0.1 # [m]
        vertical_scale = 0.005 # [m]
        border_size = 25 # [m]
        curriculum = False
        static_friction = 0.8
        dynamic_friction = 0.7
        restitution = 0.3
        # rough terrain only:
        measure_heights = True
        measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] # 1mx1.6m rectangle (without center line)
        measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
        selected = False # select a unique terrain type and pass all arguments
        terrain_kwargs = None # Dict of arguments for selected terrain
        max_init_terrain_level = 3 # starting curriculum state
        terrain_length = 8.
        terrain_width = 8.
        num_rows = 4 # number of terrain rows (levels)
        num_cols = 5 # number of terrain cols (types)
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        terrain_proportions = [1, 0., 0, 0, 0]
        # trimesh only:
        slope_treshold = 0.75 # slopes above this threshold will be corrected to vertical surfaces
        difficulty = [0.25, 0.5, 0.75, 1, 2, 2.5, 5, 7.5, 10]

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/Taks_T1/Taks_T1.urdf'
        name = "taks_t1"
        left_foot_name = "left_ankle_pitch"
        right_foot_name = "right_ankle_pitch"
        left_knee_name = 'left_knee'
        right_knee_name = 'right_knee'
        foot_name = "ankle_roll"
        penalize_contacts_on = ["elbow", 'shoulder', 'waist', 'knee', 'hip', 'neck']
        terminate_after_contacts_on = []
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False

        left_shoulder_name = "left_shoulder"
        right_shoulder_name = "right_shoulder"

        left_leg_joints = ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint']
        right_leg_joints = ['right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint']
        left_hip_joints = ['left_hip_yaw_joint']
        right_hip_joints = ['right_hip_yaw_joint']

        left_hip_roll_joints = ['left_hip_roll_joint']
        right_hip_roll_joints = ['right_hip_roll_joint']    

        left_hip_pitch_joints = ['left_hip_pitch_joint']
        right_hip_pitch_joints = ['right_hip_pitch_joint']    

        left_knee_joints = ['left_knee_joint']
        right_knee_joints = ['right_knee_joint']   

        left_shoulder_roll_joints = ['left_shoulder_roll_joint']
        right_shoulder_roll_joints = ['right_shoulder_roll_joint']    

        left_arm_joints = ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_yaw_joint', 'left_wrist_pitch_joint']
        right_arm_joints = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_yaw_joint', 'right_wrist_pitch_joint']
        waist_joints = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
        neck_joints = ['neck_yaw_joint', 'neck_roll_joint', 'neck_pitch_joint']
        knee_joints = ['left_knee_joint', 'right_knee_joint']
        ankle_joints = [ 'left_ankle_pitch_joint', 'left_ankle_roll_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint']

        keyframe_name = "keyframe"
        head_name = 'keyframe_head'
        armature = 0.01

        trunk_names = ["pelvis", "torso"]
        base_name = 'torso_link'
        tracking_body_names =  ['pelvis']

        left_upper_body_names = ['left_shoulder_pitch', 'left_elbow']
        right_upper_body_names = ['right_shoulder_pitch', 'right_elbow']
        left_lower_body_names = ['left_hip_pitch', 'left_ankle_roll', 'left_knee']
        right_lower_body_names = ['right_hip_pitch', 'right_ankle_roll', 'right_knee']

        left_ankle_names = ['left_ankle_roll']
        right_ankle_names = ['right_ankle_roll']

        density = 0.001
        angular_damping = 0.01
        linear_damping = 0.01
        max_angular_velocity = 1000.
        max_linear_velocity = 1000.
        armature = 0.01
        thickness = 0.01
        fix_base_link = False

    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.99
        soft_dof_vel_limit = 0.99
        base_height_target = 0.70
        base_height_sigma = 0.25
        tracking_dof_sigma = 0.25
        only_positive_rewards = False
        orientation_sigma = 1
        is_gaussian = True
        target_head_height = 1
        target_head_margin = 1
        target_base_height_phase1 = 0.42
        target_base_height_phase2 = 0.42
        target_base_height_phase3 = 0.60
        orientation_threshold = 0.99
        left_foot_displacement_sigma = -2
        right_foot_displacement_sigma = -2
        target_dof_pos_sigma = -0.1
        tracking_sigma = 0.25

        reward_groups = ['task', 'regu', 'style', 'target']
        num_reward_groups = len(reward_groups)
        reward_group_weights = [2.5, 0.1, 1, 1]

        class scales:
            task_orientation = 1
            task_head_height = 1

    class constraints( LeggedRobotCfg.rewards ):
        is_gaussian = True
        target_head_height = 1
        target_head_margin = 1
        orientation_height_threshold = 0.9
        target_base_height = 0.42

        left_foot_displacement_sigma = -2
        right_foot_displacement_sigma = -2
        hip_yaw_var_sigma = -2
        target_dof_pos_sigma = -0.1
        post_task = False
        
        class scales:
            # regularization reward
            regu_dof_acc = -2.5e-7
            regu_action_rate = -0.01
            regu_smoothness = -0.01 
            regu_torques = -2.5e-6
            regu_joint_power = -2.5e-5
            regu_dof_vel = -1e-3
            regu_joint_tracking_error = -0.00025
            regu_dof_pos_limits = -100.0
            regu_dof_vel_limits = -1

            # style reward
            style_waist_deviation = -10
            style_hip_yaw_deviation = -10
            style_hip_roll_deviation = -10
            style_left_foot_displacement = 2.5
            style_right_foot_displacement = 2.5
            style_knee_deviation = -10
            style_shank_orientation = 10
            style_ground_parallel = 20
            style_feet_distance = -10
            style_feet_stumble = -25
            style_style_ang_vel_xy = 1
            style_shoulder_roll_deviation = -2.5
            style_neck_deviation = -1.0
            style_lower_body_deviation = 1

            # post-task reward
            target_ang_vel_xy = 10
            target_lin_vel_xy = 10
            target_feet_height_var = 2.5
            target_target_upper_dof_pos = 10
            target_target_orientation = 10
            target_target_base_height = 10

    class domain_rand:
        use_random = True

        randomize_actuation_offset = use_random
        actuation_offset_range = [-0.05, 0.05]

        randomize_motor_strength = use_random
        motor_strength_range = [0.8, 1.2]

        randomize_payload_mass = use_random
        payload_mass_range = [-1, 3]

        randomize_com_displacement = use_random
        com_displacement_range = [-0.05, 0.05]

        randomize_link_mass = use_random
        link_mass_range = [0.8, 1.2]
        
        randomize_friction = use_random
        friction_range = [0.1, 1]
        
        randomize_restitution = use_random
        restitution_range = [0.0, 1.0]
        
        randomize_kp = use_random
        kp_range = [0.8, 1.2]
        
        randomize_kd = use_random
        kd_range = [0.8, 1.2]

        randomize_initial_joint_pos = True
        initial_joint_pos_scale = [0.8, 1.2]
        initial_joint_pos_offset = [-0.1, 0.1]
        
        push_robots = False
        push_interval_s = 10
        max_push_vel_xy = 0.5

        delay = use_random
        max_delay_timesteps = 5

    class curriculum:
        pull_force = True
        force = 80
        threshold_height = 0.9
        dof_vel_limit = 300
        base_vel_limit = 20
        no_orientation = False

    class sim:
        dt =  0.005
        substeps = 1
        gravity = [0., 0. ,-9.81]  # [m/s^2]
        up_axis = 1  # 0 is y, 1 is z
        class physx:
            num_threads = 10
            solver_type = 1  # 0: pgs, 1: tgs
            num_position_iterations = 8
            num_velocity_iterations = 1
            contact_offset = 0.01  # [m]
            rest_offset = 0.0   # [m]
            bounce_threshold_velocity = 0.5 # [m/s]
            max_depenetration_velocity = 1.0
            max_gpu_contact_pairs = 2**23
            default_buffer_size_multiplier = 5
            contact_collection = 2 # 0: never, 1: last sub-step, 2: all sub-steps (default=2)


class TaksT1CfgPPO( LeggedRobotCfgPPO ):
    runner_class_name = 'OnPolicyRunner'
    class policy:
        init_noise_std = 0.8
        actor_hidden_dims = [512, 256, 128]
        critic_hidden_dims = [512, 256]
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
        # smoothness
        value_smoothness_coef = 0.25
        smoothness_upper_bound = 1.0
        smoothness_lower_bound = 0.1
    
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        save_interval = 50
        experiment_name = 'taks_t1_wall'
        algorithm_class_name = 'PPO'
        init_at_random_ep_len = True
        max_iterations = 7500
