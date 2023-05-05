// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveMotorGyroFancy extends CommandBase {
  private MotorSubsystem motorSubsystem;
  private double threshold = 10;
  private double proportionalGain = 0.002;

  /** Creates a new DriveMotorGyro. */
  public DriveMotorGyroFancy(MotorSubsystem motorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.motorSubsystem = motorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = motorSubsystem.getGyroAngle();
    double output = proportionalGain * angle;

    if (output > 0.2) { output = 0.2;}
    if (output < -0.2) {output = -0.2;}

    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("output", output);
    if (angle > threshold) {
      motorSubsystem.set(output);
    } else if (angle < -threshold) {
      motorSubsystem.set(-output);
    } else {
      motorSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
