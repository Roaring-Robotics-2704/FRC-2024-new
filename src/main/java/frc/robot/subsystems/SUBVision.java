// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SUBVision extends SubsystemBase {

  
    Double CAMERA_HEIGHT_METERS = Units.inchesToMeters(17.5);
  double TARGET_HEIGHT_METERS = Units.inchesToMeters(21);
  double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-5);

      
    public PhotonCamera camera = new PhotonCamera("PiCam");
AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(10.5), 0.0, Units.inchesToMeters(6)), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

// Construct PhotonPoseEstimator
PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.CLOSEST_TO_LAST_POSE, camera, robotToCam);

  public SUBVision() {}

  @Override
  public void periodic() {


    var res = camera.getLatestResult();
        if (res.hasTargets()) {
            var imageCaptureTime = res.getTimestampSeconds();
            var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
            // var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
            // m_poseEstimator.addVisionMeasurement(
            //         camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);


    if (camera.getLatestResult().hasTargets()) {
      camera.setLED(VisionLEDMode.kOff);
    }else {
      camera.setLED(VisionLEDMode.kOff);
    }}
    // This method will be called once per scheduler run
  }/*
  public double getSkew() {
    if(camera.getLatestResult().hasTargets()) {
    return camera.getLatestResult().getBestTarget().getSkew();} else {return 0;}
  }
  public double getRotation() {
    if(camera.getLatestResult().hasTargets()) {
    return camera.getLatestResult().getBestTarget().getYaw();} else {return 0;}
  }*/
    public double getDistance() {
if(camera.getLatestResult().hasTargets()) {
    return PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(camera.getLatestResult().getBestTarget().getPitch()));

  } else {return 2;}  }
}
