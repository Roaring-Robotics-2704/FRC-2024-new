// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUBClimb;
import frc.utils.RoaringUtils.DeadzoneUtils;

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
    climb.changeLeftHookPosition(-((xbox.leftBumper().getAsBoolean() ? 0.5 : 0)-(xbox.leftTrigger().getAsBoolean() ? 0.5 : 0)));
    climb.changeRightHookPosition(-((xbox.rightBumper().getAsBoolean() ? 0.5 : 0)-(xbox.rightTrigger().getAsBoolean() ? 0.5 : 0)));

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
