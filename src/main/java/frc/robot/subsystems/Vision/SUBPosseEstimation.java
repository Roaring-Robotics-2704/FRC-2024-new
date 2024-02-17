package frc.robot.subsystems.Vision;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.io.IOException;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.SUBSwerveDrive;
public class SUBPosseEstimation extends SubsystemBase {
    private final PhotonCamera photoncamera;
    private final SUBSwerveDrive drivesubsytem;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    public static final Vector <N3> stateStdDevss = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(5));
    public static final Vector <N3> visionMeasumenstStdDevs = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(10));
    
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d feild2d = new Field2d();

    private double previousPipeLineTimestamp = 0;
    
    private PhotonPoseEstimator photonPoseEstimator2;

    public SUBPosseEstimation(PhotonCamera photonCamera, SUBSwerveDrive driveSubsystem){
        this.photoncamera = photonCamera;
        this.drivesubsytem = driveSubsystem; 
        AprilTagFieldLayout layout;
        try{
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            var alliance = DriverStation.getAlliance();
        }  catch (IOException e){
            DriverStation.reportError("Failled to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
        }
        this.aprilTagFieldLayout = layout;
        ShuffleboardTab tab = Shuffleboard.getTab("vision");

        poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,Rotation2d.fromDegrees(driveSubsystem.getHeading()), driveSubsystem.getPosition(),new Pose2d(),stateStdDevss,visionMeasumenstStdDevs);
        this.photonPoseEstimator2 = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS,photonCamera,Constants.VisonConstants.APRILTAG_CAMERA_TO_ROBOT);
        // do NOT use multitags 
        tab.addString("Pose",this::getFormattedPose).withPosition(0,0).withSize(2,0);
        tab.add("feild",feild2d).withPosition(2,0).withSize(6,4);
    }
@ Override
public void periodic(){
    var res = photoncamera.getLatestResult();
    if(res.hasTargets()){
        var imageCaptureTime = res.getTimestampSeconds();
        var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
        var camPose = aprilTagFieldLayout.getTagPose(res.getBestTarget().getFiducialId()).get().getTranslation().transfromBy(camToTargetTrans.inverse());
        PoseEstimator.addVisionMesurment(camPose.transfromBy(Constants.VisonConstants.APRILTAG_CAMERA_TO_ROBOT).toPose2d(),imageCaptureTime);
    }
    poseEstimator.addVisionMeasurement(visionMeasurment.toPose2d(),imageCaptureTime);

PoseEstimator.update(
    Rotation2d.fromDegrees(SUBSwerveDrive.m_gyro.getAngle)
);
}
}
