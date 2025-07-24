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
        self.run_speed = 0.001
        self.min_position = 0.4
        self.max_position = 0.6
        self.position_A = 0.5
        self.set_destination(self.position_A)

    def periodic(self): # Special function called periodically by the robot
        wpilib.SmartDashboard.putNumber(constants.DASHBOARD_TITLES["GENERAL_SERVO_POSITION"], self.position)


    ## Methods
    def set_destination(self, destination):
        self.position = destination
        self.servo.set(self.position)

    def adjust_position(self, direction):
        this_run_speed = self.run_speed * direction
        new_position_capped_at_min = min(self.position + this_run_speed, self.max_position)
        new_position_within_caps = max(new_position_capped_at_min, self.min_position)
        self.set_destination(new_position_within_caps)


    ## Commands
    def run_to_min_position_command(self):
        return commands2.cmd.runOnce(lambda: self.set_destination(self.min_position), self)

    def run_to_max_position_command(self):
        return commands2.cmd.runOnce(lambda: self.set_destination(self.max_position), self)

    def run_to_A_position_command(self):
        return commands2.cmd.runOnce(lambda: self.set_destination(self.position_A), self)

    def adjust_servo_ahead_command(self):
        return commands2.cmd.run(lambda: self.adjust_position(1), self)

    def adjust_servo_reverse_command(self):
        return commands2.cmd.run(lambda: self.adjust_position(-1), self)

