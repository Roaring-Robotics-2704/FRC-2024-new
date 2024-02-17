package frc.robot.commands.Vision;

import frc.robot.commands.Drive.CMDDrive;
import frc.robot.Constants.*;

import org.photonvision.PhotonUtils;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drive.SUBSwerveDrive;
public class CMDDriveToTarget extends Command {
double forwardspeed;
double Rotation;
PhotonCamera camera = new PhotonCamera("PiCam");

}
