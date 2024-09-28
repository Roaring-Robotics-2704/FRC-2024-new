// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUBClimb;

public class CMDClimb extends Command {
  /** Creates a new CMDClimb. */
  CommandXboxController xbox = RobotContainer.getDriverController1();
  SUBClimb climb;
  public CMDClimb(SUBClimb climb) {
  this.climb = climb;
  addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.changeLeftHookPosition(((xbox.getHID().getLeftBumper() ? 0.7 : 0)- ((xbox.getHID().getLeftTriggerAxis()==1) ? 0.7 : 0)));
    climb.changeRightHookPosition(-((xbox.getHID().getRightBumper() ? 0.7 : 0)-((xbox.getHID().getRightTriggerAxis()==1) ? 0.7 : 0)));

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
