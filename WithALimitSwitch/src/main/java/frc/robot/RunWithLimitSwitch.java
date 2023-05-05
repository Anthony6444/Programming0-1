// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunWithLimitSwitch extends CommandBase {
  private MotorSubsystem motorSubsystem;

  /** Creates a new RunWithLimitSwitch. */
  public RunWithLimitSwitch(MotorSubsystem motorSubsystem) {
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
    SmartDashboard.putBoolean("LimitSwitch", motorSubsystem.getLimitSwitch());
    if (motorSubsystem.getLimitSwitch()) {
      motorSubsystem.stop();
    } else {
      motorSubsystem.set(0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
