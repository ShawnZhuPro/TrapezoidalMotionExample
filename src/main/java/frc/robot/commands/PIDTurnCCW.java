// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDTurnCCW extends CommandBase {
  DriveTrain dt; // Declare a reference to the DriveTrain subsystem.

  // To determine the kP constant, you will take your motor power and divide it by the setpoint (0.6/90)
  PIDController pid = new PIDController(0.2, 0.0, 0.04);

  double angle; // Declare a variable to determine the turn angle 
  int motorSign; // Declare a variable to determine the direction of the turn.

  public PIDTurnCCW(double angle, DriveTrain dt) {
    this.dt = dt; // Initialize the DriveTrain reference.
    this.angle = angle; // Initialize the desired turn angle.
    pid.setTolerance(5.0); // Set an absolute error tolerance of 5 degrees 
    // pid.setSetpoint(angle);  setSetpoint is for exact values

    // Determine the direction of the turn based on the sign of 'angle'.
    if (angle > 0) {
      motorSign = 1; //Counter-clockwise turn
    } else if (angle <= 0) {
      motorSign = -1; // clockwise turn
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt); // Specify that this command requires the 'DriveTrain' subsystem.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.zeroHeading(); // The heading represents the current orientation or angle of a robot or vehicle relative to a reference point or direction, typically measured in degrees.
    dt.tankDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate the output using the PID controller to control the turn.
    double output = pid.calculate(dt.getHeading(), angle);

    // Adjust the motor voltages to execute the turn.
    dt.tankDrive(-output * motorSign, output * motorSign);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0); // Stop the motors when the command ends.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint(); // This command is finished when the PID controller reaches its setpoint.
  }
}
