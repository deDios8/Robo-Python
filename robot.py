import wpilib
import commands2
from subsystems.SS_GeneralMotor import SS_GeneralMotor
from subsystems.SS_GeneralServo import SS_GeneralServo
from subsystems.SS_EncodedMotor import SS_EncodedMotor


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        RobotContainer()

    def teleopPeriodic(self):
        commands2.CommandScheduler.getInstance().run()


class RobotContainer:
    def __init__(self):
        self.initialize_subsystems()
        self.keybindings()

    def initialize_subsystems(self):
        self.controller = commands2.button.CommandXboxController(0)
        self.ss_general_motor = SS_GeneralMotor()
        self.ss_general_servo = SS_GeneralServo()
        self.ss_encoded_motor = SS_EncodedMotor()

    def keybindings(self):
        self.controller.x().whileTrue(self.ss_general_motor.run_forward_command2())
        self.controller.a().onFalse(self.ss_general_servo.run_to_min_position_command())
        self.controller.b().onFalse(self.ss_general_servo.run_to_max_position_command())
        self.controller.y().onFalse(self.ss_general_servo.run_to_A_position_command())
        
        self.controller.rightBumper().whileTrue(self.ss_general_servo.adjust_servo_ahead_command())
        self.controller.leftBumper().whileTrue(self.ss_general_servo.adjust_servo_reverse_command())

        self.controller.povUp().whileTrue(self.ss_encoded_motor.run_forward_command())
        self.controller.povDown().onTrue(self.ss_encoded_motor.stop_motor_command())
        self.controller.povLeft().onTrue(self.ss_encoded_motor.go_to_destination_A_command())
        self.controller.povRight().onTrue(self.ss_encoded_motor.go_to_destination_B_command())
        

if __name__ == "__main__":
    wpilib.run(MyRobot)