// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveMotorGyro extends CommandBase {
  private MotorSubsystem motorSubsystem;
  private double threshold = 10;  

  /** Creates a new DriveMotorGyro. */
  public DriveMotorGyro(MotorSubsystem motorSubsystem) {
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
    if (motorSubsystem.getGyroAngle() > threshold) {
      motorSubsystem.set(0.2);
    } else if (motorSubsystem.getGyroAngle() < -threshold) {
      motorSubsystem.set(-0.2);
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
