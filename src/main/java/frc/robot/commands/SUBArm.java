// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SUBArm extends SubsystemBase {
  /** Creates a new SUBArm. */
   CANSparkMax armMotor1 = new CANSparkMax(Constants.ArmConstants.kArmMotor1, MotorType.kBrushless);
   CANSparkMax armMotor2 = new CANSparkMax(Constants.ArmConstants.kArmMotor2, MotorType.kBrushless);

  public SUBArm(

  ) {

armMotor2.follow(armMotor1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
