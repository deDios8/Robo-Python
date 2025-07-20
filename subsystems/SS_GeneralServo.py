import wpilib
import commands2
import constants
import wpimath.units


class SS_GeneralServo(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.servo = wpilib.Servo(constants.PWM_CHANNELS["GENERAL_SERVO"])
        pulse_width_min = wpimath.units.microseconds(600)
        pulse_width_center = wpimath.units.microseconds(1500)
        pulse_width_max = wpimath.units.microseconds(2400)
        deadband = wpimath.units.microseconds(8)
        self.servo.setBounds(pulse_width_min, deadband, pulse_width_center, deadband, pulse_width_max)
        self.min_position = 0.0
        self.position_A = 0.5
        self.max_position = 0.9
        self.position = self.position_A
        self.destination = self.position_A
        self.servo.set(self.position_A)
        self.run_speed = 0.001


    def periodic(self): # Special function called periodically by the robot
        self.position = self.servo.getPosition()
        wpilib.SmartDashboard.putNumber(constants.DASHBOARD_TITLES["GENERAL_SERVO_POSITION"], self.position)
        wpilib.SmartDashboard.putNumber(constants.DASHBOARD_TITLES["GENERAL_SERVO_DESTINATION"], self.destination)

    def run_to_position(self, destination):
        print(f"Running to destination: {destination}")
        self.servo.set(destination)
    def run_to_min_position_command(self):
        return commands2.cmd.runOnce(lambda: self.run_to_position(self.min_position), self)
    def run_to_A_position_command(self):
        return commands2.cmd.runOnce(lambda: self.run_to_position(self.position_A), self)
    def run_to_max_position_command(self):
        return commands2.cmd.runOnce(lambda: self.run_to_position(self.max_position), self)

    def run_direction(self, direction):
        print(f"Run direction to: {self.position}")
        run_speed = self.run_speed * direction
        self.destination = min(self.position + run_speed, self.max_position)
        self.destination = max(self.destination, self.min_position)
        self.run_to_position(self.destination)
    def run_forward_command(self):
        return commands2.cmd.runOnce(lambda: self.run_direction(1), self)
    def run_backward_command(self):
        return commands2.cmd.runOnce(lambda: self.run_direction(-1), self)

    def stop_run_command(self):
        print(f"Stopping run at: {self.position}")
        self.destination = self.position
        return commands2.cmd.runOnce(lambda: self.run_to_position(self.position), self)
