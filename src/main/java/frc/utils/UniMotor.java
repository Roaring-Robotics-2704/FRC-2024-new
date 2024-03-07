// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class UniMotor {
  public CANSparkMax sparkMax;
  public SparkPIDController pid;
  public TalonSRX talonSRX;

  public enum UniMotorType {
    TalonSRX,
    SparkMAX
  }

  public enum UniMotorMode {
    Brake,
    Coast
  }

  UniMotorType type;
    
  public UniMotor(int CANID, UniMotorType type) {
    if (type == UniMotorType.SparkMAX) {
      sparkMax = new CANSparkMax(CANID, MotorType.kBrushless);
      pid = sparkMax.getPIDController();
      this.type = type;
    } else if (type == UniMotorType.TalonSRX) {
      talonSRX = new TalonSRX(CANID);
      this.type = type;
    }
    
    killMotor();
  }

  public void setSmartCurrentLimit(int CurrentLimit) {
    if (type == UniMotorType.SparkMAX){
      sparkMax.setSmartCurrentLimit(CurrentLimit);
    } else if (type == UniMotorType.TalonSRX) {
      talonSRX.configContinuousCurrentLimit(CurrentLimit);
    }
  }

  public void follow(UniMotor motor) {
    if (type == UniMotorType.SparkMAX){
      sparkMax.follow(motor.sparkMax);
    } else if (type == UniMotorType.TalonSRX) {
      talonSRX.follow(motor.talonSRX);
    }
  }

  public void set(double percent) {
    if (type == UniMotorType.SparkMAX){
      sparkMax.set(percent);
    } else if (type == UniMotorType.TalonSRX) {
      talonSRX.set(TalonSRXControlMode.PercentOutput,percent);
    }
  }

  public void setInverted(boolean invert) {
    if (type == UniMotorType.SparkMAX){
      sparkMax.setInverted(invert);
    } else if (type == UniMotorType.TalonSRX) {
      talonSRX.setInverted(invert);
    } 
  }

  private void killMotor() {
    if (type == UniMotorType.TalonSRX) {
      //sparkMax.close();
    } 
  }

  /**
   * WORKS ONLY ON SPARK MAX
   * @param velo
   */
  public void setVelocitySpark(double velo) {
    if (type == UniMotorType.SparkMAX){
      pid.setReference(velo,ControlType.kVelocity);
    } else if (type == UniMotorType.TalonSRX) {
      talonSRX.set(TalonSRXControlMode.PercentOutput,velo);
    } 
  }

  public void setIdleMode(UniMotorMode mode) {
    if (type == UniMotorType.SparkMAX){
      sparkMax.setIdleMode(mode  == UniMotorMode.Brake ? IdleMode.kBrake : IdleMode.kCoast);
    } else if (type == UniMotorType.TalonSRX) {
      talonSRX.setNeutralMode(mode == UniMotorMode.Brake ? NeutralMode.Brake : NeutralMode.Coast);  
    } 
  }

  public void setPID(double P, double I, double D) {
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
  }
}
