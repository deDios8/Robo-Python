import wpilib
import commands2 #installed from robotpy-commands-v2
from subsystems.SS_GeneralMotor import SS_GeneralMotor
from subsystems.SS_GeneralServo import SS_GeneralServo


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        RobotContainer()

    def teleopPeriodic(self):
        commands2.CommandScheduler.getInstance().run()


class RobotContainer:
    def __init__(self):
        self.initialize_subsystems()

        self.controller = commands2.button.CommandXboxController(0)
        self.keybindings()

    def initialize_subsystems(self):
        self.ss_general_motor = SS_GeneralMotor()
        self.ss_general_servo = SS_GeneralServo()

    def keybindings(self):
        self.controller.x().onTrue(self.ss_general_motor.run_forward_command())
        self.controller.x().onFalse(self.ss_general_motor.stop_motor_command())
        self.controller.y().onTrue(self.ss_general_motor.run_backward_command())
        self.controller.y().onFalse(self.ss_general_motor.stop_motor_command())

        self.controller.a().onFalse(self.ss_general_servo.run_to_min_position_command())
        self.controller.b().onFalse(self.ss_general_servo.run_to_max_position_command())
        self.controller.a().onTrue and self.controller.b().onTrue(self.ss_general_servo.run_to_A_position_command())

        self.controller.povUp().onTrue(self.complicatedCommand)
        #self.controller.povDown().onTrue(self.moveBackwardCommand)
        #self.controller.povLeft().onTrue(self.turnLeftCommand)
        #self.controller.povRight().onTrue(self.turnRightCommand)

        

        self.controller.rightBumper().whileTrue(self.ss_general_servo.adjust_servo_ahead_command())
        self.controller.leftBumper().whileTrue(self.ss_general_servo.adjust_servo_reverse_command())
        

if __name__ == "__main__":
    wpilib.run(MyRobot)