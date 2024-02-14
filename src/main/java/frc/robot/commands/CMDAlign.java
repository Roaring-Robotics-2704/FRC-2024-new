  //Copyright (c) FIRST and other WPILib contributors.
  //Open Source Software; you can modify and/or share it under the terms of
  //the WPILib BSD license file in the root directory of this project.

 package frc.robot.commands;

 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonUtils;

 import edu.wpi.first.math.controller.PIDController;
 import edu.wpi.first.math.kinematics.ChassisSpeeds;
 import edu.wpi.first.math.util.Units;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
 import frc.robot.RobotContainer;
 import frc.robot.subsystems.DriveSubsystem;
 import frc.robot.subsystems.SUBVision;

 public class CMDAlign extends Command {
       PIDController translationPID = new PIDController(0.1, 0, 0);
       PIDController rotationPID = new PIDController(0.1, 0, 0);
        DriveSubsystem m_drive = RobotContainer.m_robotDrive;
        SUBVision m_vision =  RobotContainer.m_SUBVision;


          Double CAMERA_HEIGHT_METERS = Units.inchesToMeters(17.5);
   double TARGET_HEIGHT_METERS = Units.inchesToMeters(21);
   double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-5);

   /** Creates a new SUBVision. */
       PhotonCamera camera = m_vision.camera;
   /** Creates a new CMDAlign. */
   public CMDAlign() {
     addRequirements(RobotContainer.m_SUBVision,RobotContainer.m_robotDrive);
      //Use addRequirements() here to declare subsystem dependencies.
   }

    //Called when the command is initially scheduled.
   @Override
   public void initialize() {}

    //Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
     if (camera.getLatestResult().hasTargets()) {
     double y = translationPID.calculate(m_vision.getDistance(),1);
     //double x = translationPID.calculate(m_vision.getSkew(),0)
     double z = rotationPID.calculate(camera.getLatestResult().getBestTarget().getYaw(),0);
     m_drive.driveRobotRelative(new ChassisSpeeds(-y,0,z));
     } else {     m_drive.driveRobotRelative(new ChassisSpeeds(0,0,0));
}
     if (camera.getLatestResult().hasTargets()) {
     SmartDashboard.putNumber("Distance",  PhotonUtils.calculateDistanceToTargetMeters(
                                 CAMERA_HEIGHT_METERS,
                                 TARGET_HEIGHT_METERS,
                                 CAMERA_PITCH_RADIANS,
                                 Units.degreesToRadians(camera.getLatestResult().getBestTarget().getPitch()))); 
         SmartDashboard.putNumber("Angle", camera.getLatestResult().getBestTarget().getYaw()); 
     }
     SmartDashboard.putBoolean("has target", camera.getLatestResult().hasTargets());
     }

    //Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {}

    //Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }
 }
