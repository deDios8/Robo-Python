import phoenix6
import commands2 #installed from robotpy-commands-v2
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds
from phoenix6.swerve import SwerveDrivetrain, SwerveModuleConstants, SwerveDrivetrainConstants, ClosedLoopOutputType, SteerFeedbackType
from phoenix6.configs import Pigeon2Configuration
from phoenix6.signals import NeutralModeValue

class SS_SwerveCTRE(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        # Define drivetrain and module constants
        self.drivetrain_constants = SwerveDrivetrainConstants(
            can_bus_name="DriveTrainCANivore", 
            pigeon2_id=0,
            pigeon2_configs=Pigeon2Configuration()
        )
       
        # Helper function to create a swerve module
        def create_swerve_module(drive_motor_id, steer_motor_id, encoder_id, cancoder_offset=0, 
                                 drive_p=.1, drive_i=0, drive_d=0, drive_s=0, drive_v=0.124,
                                 steer_p=50, steer_i=0, steer_d=0.1, steer_v=0.124, steer_s=0, steer_a=0,
                                 ):
            # maybe we should use swerve_module_constants.py SwerveModuleConstantsFactory
            return SwerveModuleConstants(
                drive_motor_id=drive_motor_id,
                steer_motor_id=steer_motor_id,
                encoder_id=encoder_id,
                cancoder_offset=phoenix6.units.rotation(cancoder_offset), # phoenix6.units.rotation
                drive_motor_gains=phoenix6.configs.Slot0Configs()
                    .with_k_p(drive_p).with_k_i(drive_i).with_k_d(drive_d)
                    .with_k_s(drive_s).with_k_v(drive_v),
                steer_motor_gains=phoenix6.configs.Slot0Configs()
                    .with_k_p(steer_p).with_k_i(steer_i).with_k_d(steer_d)
                    .with_k_s(steer_s).with_k_v(steer_v).with_k_a(steer_a),
                drive_motor_inverted=False,
                steer_motor_inverted=False,
                drive_motor_gear_ratio=5.90278,
                steer_motor_gear_ratio=12.8,
                coupling_gear_ratio=3.125, # Every 1 rotation on azimuth motor causes this many rotations on drive motor
                wheel_radius=phoenix6.units.inch(2),
                speed_at12_volts=phoenix6.units.meters_per_second(5.41),
                slip_current=phoenix6.units.ampere(120),
                steer_motor_closed_loop_output=ClosedLoopOutputType.VOLTAGE,
                drive_motor_closed_loop_output=ClosedLoopOutputType.VOLTAGE,
                steer_motor_type=phoenix6.hardware.TalonFX, # TalonFX control unit is frequently used with Kraken motors
                drive_motor_type=phoenix6.hardware.TalonFX,
                cancoder_initial_configs=phoenix6.configs.CANcoderConfiguration(),
                drive_motor_initial_configs=phoenix6.configs.TalonFXConfiguration()
                    .with_current_limits(phoenix6.units.ampere(40))
                    .with_motor_output(neutral_mode=NeutralModeValue.BRAKE), # or COAST
                steer_motor_initial_configs=phoenix6.configs.TalonFXConfiguration()
                    .with_current_limits(phoenix6.units.ampere(40))
                    .with_motor_output(neutral_mode=NeutralModeValue.COAST), # or BRAKE
                feedback_source=SteerFeedbackType.REMOTE_CANCODER,
                # Used for simulation
                steer_inertia=phoenix6.units.kilogram_square_meter(0.00001),
                drive_inertia=phoenix6.units.kilogram_square_meter(0.001),
                steer_friction_voltage=phoenix6.units.volt(.2),
                drive_friction_voltage=phoenix6.units.volt(.2),
            )
        
        # Define modules
        self.modules = [
            create_swerve_module(3, 4, 5, -0.178466796875),  # Front Left
            create_swerve_module(0, 1, 2, 0.07421875),  # Front Right
            create_swerve_module(6, 7, 8, -0.011962890625),  # Back Left
            create_swerve_module(9, 10, 11, 0.4169921875),  # Back Right
        ]

        # Initialize the swerve drivetrain with the defined constants and modules
        self.drivetrain = SwerveDrivetrain(
            drive_motor_type = phoenix6.hardware.TalonFX, # TalonFX is frequently used with Kraken motors
            steer_motor_type = phoenix6.hardware.TalonFX, 
            encoder_type = phoenix6.hardware.CANcoder,
            drivetrain_constants = self.drivetrain_constants,
            modules = self.modules,
        )

        # Define kinematics
        half_length_between_wheels = phoenix6.units.inch(13.5)
        half_width_between_wheels = phoenix6.units.inch(10.75)
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(half_length_between_wheels, half_width_between_wheels),  # Front Left
            Translation2d(half_length_between_wheels, -half_width_between_wheels), # Front Right
            Translation2d(-half_length_between_wheels, half_width_between_wheels), # Back Left
            Translation2d(-half_length_between_wheels, -half_width_between_wheels), # Back Right
        )

        # Initialize odometry
        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.drivetrain.get_state().raw_heading, # Use the gyro angle from your SwerveDrivetrain
            tuple(module.get_position() for module in self.drivetrain.modules),
            Pose2d(),
        )


    def periodic(self):
        # Update odometry in periodic
        self.odometry.update(
            self.drivetrain.get_state().raw_heading, #TODO not sure if get_rotation3d is ok, because no get_pose2d was available
            tuple(module.get_position() for module in self.drivetrain.modules)
        )

    def set_module_states(self, module_states):
        # Set individual module states (e.g., speed and angle)
        self.kinematics.desaturateWheelSpeeds(module_states, self.drivetrain.max_speed) #TODO: not sure if this can represnet max_speed of the modules
        for i, module_state in enumerate(module_states):
            # optimized_module_state = SwerveModuleState.optimize(module_state, self.drivetrain.modules[i].get_angle())
            optimized_module_state = SwerveModuleState.optimize(module_state, self.drivetrain.modules[i].get_current_state().angle)
            self.drivetrain.modules[i].set(optimized_module_state.speed, optimized_module_state.angle.radians())

    def get_pose(self):
        return self.odometry.getEstimatedPosition() # TODO: unknown if this is the correct function

    def reset_odometry(self, pose):
        self.odometry.resetPosition( #TODO: The parameters here are not correct, need to check
            pose,
            self.drivetrain.get_state().raw_heading,
            tuple(module.get_position() for module in self.drivetrain.modules),
        )


    def drive_field_relative(self, x_speed, y_speed, rot_speed):
        # Implement field-relative driving
        # .fromFieldRelativeSpeeds is the only difference from drive_robot_relative below
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds( 
            x_speed, y_speed, rot_speed, self.odometry.get_pose().rotation() )
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        self.set_module_states(module_states)
    def drive_field_relative_command(self, x_speed, y_speed, rot_speed):
        # Create a command to drive field-relative
        return commands2.Command(lambda: self.drive_field_relative(x_speed, y_speed, rot_speed))
    
    def drive_robot_relative(self, x_speed, y_speed, rot_speed):
        # Implement robot-relative driving
        chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot_speed)
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        self.set_module_states(module_states)
    def drive_robot_relative_command(self, x_speed, y_speed, rot_speed):
        # Create a command to drive robot-relative
        return commands2.Command(lambda: self.drive_robot_relative(x_speed, y_speed, rot_speed))
    