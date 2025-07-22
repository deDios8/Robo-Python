import wpilib
import commands2
import constants
from commands2 import SequentialCommandGroup, WaitCommand

class SS_GeneralMotor(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.spark_motor = wpilib.PWMSparkMax(constants.PWM_CHANNELS["GENERAL_MOTOR"])
        self.spark_motor.setSafetyEnabled(False)
        self.is_running = False
        self.is_running_timed = False
        self.speed = .6

    def periodic(self): # Special function called periodically by the robot
        wpilib.SmartDashboard.putBoolean(constants.DASHBOARD_TITLES["GENERAL_MOTOR_RUNNING"], self.is_running)
        wpilib.SmartDashboard.putBoolean(constants.DASHBOARD_TITLES["GENERAL_MOTOR_RUNNING_SECONDS"], self.is_running_timed)


    def run_forward(self):
        self.spark_motor.set(self.speed)
        self.is_running = True
    def run_forward_command(self):
        return commands2.cmd.runOnce(self.run_forward, self)

    def stop_motor(self):
        self.spark_motor.stopMotor()
        self.is_running = False
    def stop_motor_command(self):
        return commands2.cmd.runOnce(self.stop_motor, self)
    
    def run_for_3_seconds_command(self):
        self.is_running_timed = True
        command_group = SequentialCommandGroup(
            self.run_forward_command(),
            WaitCommand(3),
            self.stop_motor_command()
        )
        self.is_running_timed = False
        return command_group
    




