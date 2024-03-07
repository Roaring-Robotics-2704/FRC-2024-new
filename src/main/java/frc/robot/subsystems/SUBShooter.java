// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.Constants.LauncherConstants.*;

<<<<<<< Updated upstream
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUBShooter extends SubsystemBase {
  WPI_TalonSRX m_launchWheel;
  WPI_TalonSRX m_feedWheel;

  /** Creates a new Launcher. */
  public SUBShooter() {
    m_launchWheel = new WPI_TalonSRX(kLauncherID);
    m_feedWheel = new WPI_TalonSRX(kFeederID);

    m_launchWheel.configContinuousCurrentLimit(kLauncherCurrentLimit);
    m_feedWheel.configContinuousCurrentLimit(kFeedCurrentLimit);
=======
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LauncherConstants;
import frc.robot.RobotContainer.RobotMode;
import frc.utils.UniMotor;
import frc.utils.UniMotor.UniMotorMode;
import frc.utils.UniMotor.UniMotorType;
import com.revrobotics.CANSparkLowLevel.MotorType.*;

public class SUBShooter extends SubsystemBase {

  CANSparkMax m_launchWheel1;
  CANSparkMax m_launchWheel2;

  CANSparkMax m_feedWheel;
  SparkPIDController pid;
  RelativeEncoder encoder;
  Double velocity;

  /** Creates a new Launcher. */
  public SUBShooter() {
    m_launchWheel1 = new CANSparkMax(kLauncherID1, MotorType.kBrushless);
      m_launchWheel2 = new CANSparkMax(kLauncherID2, MotorType.kBrushless);
      m_feedWheel = new CANSparkMax(kFeederID, MotorType.kBrushless);
      pid = m_launchWheel1.getPIDController();
      pid.setP(LauncherConstants.kP);
      pid.setI(LauncherConstants.kI);
      pid.setD(LauncherConstants.kD);
      encoder = m_launchWheel1.getEncoder();
      pid.setPositionPIDWrappingMaxInput(1);
      pid.setPositionPIDWrappingMaxInput(-1);
      pid.setPositionPIDWrappingEnabled(true);
      pid.setFeedbackDevice(encoder);
      
  }

  public void init () {
   // if (RobotContainer.getRobotMode() == RobotMode.CompBot) {
   //   m_launchWheel1 = new UniMotor(kLauncherID1,UniMotorType.SparkMAX);
   //   m_launchWheel2 = new UniMotor(kLauncherID2,UniMotorType.SparkMAX);
   //   m_feedWheel = new UniMotor(kFeederID,UniMotorType.SparkMAX);
   // } else if (RobotContainer.getRobotMode() == RobotMode.KitBot) {
      
   // }

    //m_launchWheel2.follow(m_launchWheel1);
    //m_launchWheel2.setInverted(false);
    m_launchWheel1.setSmartCurrentLimit(kLauncherCurrentLimit);
    //m_launchWheel2.setSmartCurrentLimit(kLauncherCurrentLimit);
    m_feedWheel.setSmartCurrentLimit(kFeedCurrentLimit);
    m_feedWheel.setIdleMode(IdleMode.kBrake);
    m_launchWheel1.setIdleMode(IdleMode.kCoast);
    m_launchWheel2.setIdleMode(IdleMode.kCoast);
     encoder = m_launchWheel1.getEncoder();
     pid.setFeedbackDevice(encoder);

     pid.setSmartMotionAllowedClosedLoopError(100,0);
>>>>>>> Stashed changes
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(kIntakeFeederSpeed);
          setLaunchWheel(kIntakeLauncherSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }
<<<<<<< Updated upstream
=======

public Command getIdleCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
      // When the command is initialized, set the wheels to the intake speed values
      () -> {
        setFeedWheel(0);
        setLaunchWheel(1);
      },
      // When the command stops, stop the wheels
      () -> {
        stop();
      }
    );
  }

>>>>>>> Stashed changes
  public Command getLaunchCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
<<<<<<< Updated upstream
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setFeedWheel(kLaunchFeederSpeed);
          setLaunchWheel(kLauncherSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }
  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheel.set(speed);
=======
      // When the command is initialized, set the wheels to the intake speed values
      () -> {
        setFeedWheel(1);
        setLaunchWheel(1);
      },
      // When the command stops, stop the wheels
      () -> {
        stop();
      }
    );
  }
  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunchWheel(double speed) {
    m_launchWheel1.set(speed);
    m_launchWheel2.set(speed);
    velocity = 5000* speed;
    //pid.setReference(velocity, ControlType.);
    //m_launchWheel2.set(speed);
    //m_launchWheel1.setVelocitySpark(2000);
>>>>>>> Stashed changes
  }

  // An accessor method to set the speed (technically the output percentage) of the feed wheel
  public void setFeedWheel(double speed) {
    m_feedWheel.set(speed);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_launchWheel.set(0);
    m_feedWheel.set(0);
  }
  public double getRPM() {
    return encoder.getVelocity();
  }
}