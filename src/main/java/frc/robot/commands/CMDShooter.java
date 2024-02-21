 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.

 package frc.robot.commands;

 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 import frc.robot.Constants.OIConstants;
 import frc.robot.RobotContainer;
 import frc.robot.subsystems.SUBShooter;

 public class CMDShooter extends Command {
   SUBShooter m_SubShooter;
   public static CommandXboxController xbox = new CommandXboxController(OIConstants.kDriverControllerPort);
  /** Creates a new CMDShooter. */
   public CMDShooter(SUBShooter sub) {
    // addRequirements(RobotContainer.m_SUBShooter);
     // Use addRequirements() here to declare subsystem dependencies.\
     addRequirements(sub);
     this.m_SubShooter = sub;
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
   @Override
  public void execute() {
    double feedvalue = 0;
    if (xbox.getHID().getRightBumper()) {feedvalue=feedvalue+0.2;}
     if (xbox.getHID().getLeftBumper()) {feedvalue=feedvalue-0.2;}

     m_SubShooter.setLaunchWheel(xbox.getRightTriggerAxis()-xbox.getLeftTriggerAxis());
     m_SubShooter.setFeedWheel(feedvalue);
  }

//   // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }
 }
