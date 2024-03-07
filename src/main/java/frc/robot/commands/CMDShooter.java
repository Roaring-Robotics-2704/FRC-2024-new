// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUBShooter;

public class CMDShooter extends Command {
<<<<<<< Updated upstream
  SUBShooter m_SubShooter = new SUBShooter();
  public static CommandXboxController xbox = new CommandXboxController(OIConstants.kDriverControllerPort);
=======
  private SUBShooter subShooter;
  private static CommandXboxController xbox = RobotContainer.getDriverController2();
>>>>>>> Stashed changes
  /** Creates a new CMDShooter. */
  public CMDShooter() {
    addRequirements(RobotContainer.m_SUBShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedvalue = 0;
<<<<<<< Updated upstream
    if (xbox.getHID().getRightBumper()) {feedvalue=feedvalue+0.2;}
    if (xbox.getHID().getLeftBumper()) {feedvalue=feedvalue-0.2;}

    m_SubShooter.setLaunchWheel(xbox.getRightTriggerAxis()-xbox.getLeftTriggerAxis());
    m_SubShooter.setFeedWheel(feedvalue);
=======
    double launchvalue= 0;
    if (xbox.leftTrigger().getAsBoolean()) {feedvalue=0.5;}
    if (xbox.leftBumper().getAsBoolean()) {feedvalue=-0.5; launchvalue =-.25;}
    //if (xbox.rightTrigger().getAsBoolean()) {launchvalue=1;}
    if (xbox.rightBumper().getAsBoolean()) {launchvalue=-0.5;} else {
    launchvalue = xbox.getRightTriggerAxis();}
    subShooter.setLaunchWheel(launchvalue);
    subShooter.setFeedWheel(feedvalue);
    SmartDashboard.putNumber("shooter speed", subShooter.getRPM());
>>>>>>> Stashed changes
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
