// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SUBShooter;
import frc.robot.subsystems.SUBVision;



public class CMDShooter extends Command {
  private SUBShooter subShooter;
  private SUBVision subVision;

  private static CommandXboxController xbox = RobotContainer.getDriverController2();
  /** Creates a new CMDShooter. */
  public CMDShooter(SUBShooter sub,SUBVision vision) {
    // addRequirements(RobotContainer.m_SUBShooter);
    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(sub);
    addRequirements(vision);
    this.subShooter = sub;
    this.subVision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double feedvalue = 0;
    double launchvalue= 0;
    if (xbox.getHID().getLeftTriggerAxis()==1) {feedvalue=0.7;}
    //if (xbox.rightTrigger().getAsBoolean()) {launchvalue=1;}
    if (xbox.getHID().getRightTriggerAxis()==1) {launchvalue=0.1;feedvalue=0.5; subVision.setLights(VisionLEDMode.kOff);} else {
    subVision.setLights(VisionLEDMode.kOff);}
    subShooter.setLaunchWheel(launchvalue);
    subShooter.setFeedWheel(feedvalue);
    //SmartDashboard.putNumber("shooter speed", subShooter.getRPM());
    SmartDashboard.putNumber("Upper Roller", subShooter.getUpperRPM());
    SmartDashboard.putNumber("Lower Roller", subShooter.getLowerRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
 

 public Command LaunchSequence() {
  return new RunCommand(()->
  subShooter.setLaunchWheel(1), subShooter).repeatedly().until(subShooter::bothSpeedsMAX).withTimeout(1).andThen(
    new RunCommand(()->subShooter.setWheels(1.0, 1.0),subShooter).repeatedly().withTimeout(1)
    );
 }

}
  //  if (robotChooser.getSelected() == RobotMode.KitBot) {
    //  OIDriverController2
    //  .rightBumper()
    //  .whileTrue(
    //    new PrepareLaunch(kSUBShooter)
    //    .withTimeout(LauncherConstants.kLauncherDelay)
    //    .andThen(new LaunchNote(kSUBShooter))
    //    .handleInterrupt(() -> kSUBShooter.stop())
    //  );
    //  OIDriverController2.leftBumper().whileTrue(kSUBShooter.getIntakeCommand());

  //  }
