import wpilib
import commands2
from subsystems.SS_GeneralMotor import SS_GeneralMotor


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        RobotContainer()

    def teleopPeriodic(self):
        commands2.CommandScheduler.getInstance().run()


class RobotContainer:
    def __init__(self):
        self.ss_general_motor = SS_GeneralMotor()
        self.controller = commands2.button.CommandXboxController(0)
        self.keybindings()

    def keybindings(self):
        self.controller.a().onTrue(self.ss_general_motor.run_forward_command())
        self.controller.a().onFalse(self.ss_general_motor.stop_motor_command())


if __name__ == "__main__":
    wpilib.run(MyRobot)