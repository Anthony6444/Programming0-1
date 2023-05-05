// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSubsystem extends SubsystemBase {
  private CANSparkMax motorController = new CANSparkMax(11, MotorType.kBrushless);
  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    motorController.restoreFactoryDefaults();
    gyro.setYawAxis(IMUAxis.kX);
  }

  public void set(double speed) {
    motorController.set(speed);
  }

  public void stop() {
    motorController.set(0);
  }

  public double getGyroAngle(){
    return gyro.getAngle();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
