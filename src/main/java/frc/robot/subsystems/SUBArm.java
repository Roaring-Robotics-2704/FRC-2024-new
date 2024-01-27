// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;



public class SUBArm extends SubsystemBase {
  /** Creates a new SUBArm. */
   CANSparkMax armMotor1 = new CANSparkMax(Constants.ArmConstants.kArmMotor1, MotorType.kBrushless);
   CANSparkMax armMotor2 = new CANSparkMax(Constants.ArmConstants.kArmMotor2, MotorType.kBrushless);
   RelativeEncoder encoder = armMotor1.getEncoder();
   double setpoint = 0;

   PIDController pid = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);

  public SUBArm(

  ) {

armMotor2.follow(armMotor1);

  }

  public void setPosition(double position){
    setpoint = position; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.setDefaultNumber("armP", Constants.ArmConstants.kP);
    SmartDashboard.setDefaultNumber("armI", Constants.ArmConstants.kI);
    SmartDashboard.setDefaultNumber("armD", Constants.ArmConstants.kD);
    pid.setP(SmartDashboard.getNumber("armP", 0));
    pid.setI(SmartDashboard.getNumber("armI", 0));
    pid.setD(SmartDashboard.getNumber("armD", 0));


    double motorPower = pid.calculate(encoder.getPosition(), setpoint);
    armMotor1.set(motorPower);
  }
}
