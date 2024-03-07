// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.SUBArm;
import frc.utils.RoaringUtils.DeadzoneUtils;

public class CMDArm extends Command {
  /** Creates a new CMDArm. */
  double pos= ArmConstants.kIntakePosition;
  CommandXboxController xbox = RobotContainer.getDriverController2();
  SUBArm SUBArm;
  public CMDArm(SUBArm sub) {
    this.SUBArm = sub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
// if (xbox.povUp().getAsBoolean()) {
//       SUBArm.changePosition(1);
//       //SUBArm.setPosition(SUBArm.getPosition()+ArmConstants.kIncrementAmount);
//     } else if (xbox.povDown().getAsBoolean()) {
//             SUBArm.changePosition(-1);
//       //SUBArm.setPosition(SUBArm.getPosition()-ArmConstants.kIncrementAmount);
//     } else {
//       //SUBArm.setPosition(pos);
//       SUBArm.changePosition(0);
//     }
//SUBArm.setPosition(ArmConstants.kHoldPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public void setPosition(double pos) {
    this.pos = pos;
  }
}
