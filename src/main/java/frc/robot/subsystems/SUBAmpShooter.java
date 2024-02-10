// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUBAmpShooter extends SubsystemBase {
  /** Creates a new SUBAmpShooter. */
  public SUBAmpShooter() {
  AmpShooterMotor2.follow(AmpShooterMotor1, true);
  }

    CANSparkMax AmpShooterMotor1 = new CANSparkMax(Constants.AmpShooterConstants.kAmpShooterCanId1, MotorType.kBrushless);
    CANSparkMax AmpShooterMotor2 = new CANSparkMax(Constants.AmpShooterConstants.kAmpShooterCanId2, MotorType.kBrushless);

  public void setAmpShooterMotor(double percent) {
    AmpShooterMotor1.set(percent);
    SmartDashboard.putNumber("intake power (%)", percent);
}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
