import wpilib
import commands2
import constants
from rev import *

class SS_EncodedMotor(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.motor = SparkMax(constants.CAN_CHANNELS["ENCODED_MOTOR"], SparkLowLevel.MotorType.kBrushless)
        self.controller = self.motor.getClosedLoopController()
        self.encoder = self.motor.getEncoder()
        self.position = 0
        self.encoder.setPosition(self.position)
        self.is_running = False

        self.speed_cap = .1
        self.destination_A = 10
        self.destination_B = -10


    def periodic(self): # Special function called periodically by the robot
        self.position = self.encoder.getPosition()
        wpilib.SmartDashboard.putNumber(constants.DASHBOARD_TITLES["ENCODED_MOTOR_POSITION"], self.position)
        wpilib.SmartDashboard.putBoolean(constants.DASHBOARD_TITLES["ENCODED_MOTOR_RUNNING"], self.is_running)


    ## Methods
    def run_forward(self):
        self.is_running = True
        self.motor.set(self.speed_cap)

    def stop_motor(self):
        self.is_running = False
        self.motor.stopMotor()

    def run_to_destination(self, destination):
        self.is_running = True
        self.motor.set(self.speed_cap)

    def go_to_destination(self, destination):
        self.controller.setReference(destination, 
                                     SparkBase.ControlType.kPosition, 
                                     ClosedLoopSlot.kSlot0, 
                                     0, 
                                     SparkClosedLoopController.ArbFFUnits.kVoltage)


    ## Commands
    def run_forward_command(self):
        return commands2.cmd.startEnd(self.run_forward, self.stop_motor, self)
    
    def stop_motor_command(self):
        return commands2.cmd.runOnce(self.stop_motor, self)

    def run_to_destination_A_command(self):
        return commands2.cmd.runOnce(lambda: self.run_to_destination(self.destination_A), self)
    
    def run_to_destination_B_command(self):
        return commands2.cmd.runOnce(lambda: self.run_to_destination(self.destination_B), self)
    
    def go_to_destination_A_command(self):
        return commands2.cmd.runOnce(lambda: self.go_to_destination(self.destination_A), self)
    
    def go_to_destination_B_command(self):  
        return commands2.cmd.runOnce(lambda: self.go_to_destination(self.destination_B), self)
    
