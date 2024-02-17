package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUBVision extends SubsystemBase {

  
    Double CAMERA_HEIGHT_METERS = Units.inchesToMeters(17.5);
  double TARGET_HEIGHT_METERS = Units.inchesToMeters(21);
  double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-5);

      
    public PhotonCamera camera = new PhotonCamera("PiCam");



  public SUBVision() {}

  @Override
  public void periodic() {
    if (camera.getLatestResult().hasTargets()) {
      camera.setLED(VisionLEDMode.kOff);
    }else {
      camera.setLED(VisionLEDMode.kOff);
    }
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

  } else {return 1;}  }
}