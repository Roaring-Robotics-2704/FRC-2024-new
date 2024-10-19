//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

public class SUBDrive extends SubsystemBase {
  // Create MAXSwerveModules
  public final ADIS16470_IMU kGyro = new ADIS16470_IMU();
 
  private final SUBMAXSwerveModule kFrontLeft = new SUBMAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset
  );

  private final SUBMAXSwerveModule kFrontRight = new SUBMAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset
  );

  private final SUBMAXSwerveModule kRearLeft = new SUBMAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset
  );

  private final SUBMAXSwerveModule kRearRight = new SUBMAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset
  );

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry swerveDriveOdometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(kGyro.getAngle(IMUAxis.kY)),
    new SwerveModulePosition[] {
      kFrontLeft.getPosition(),
      kFrontRight.getPosition(),
      kRearLeft.getPosition(),
      kRearRight.getPosition()
    }
  );

  private Field2d field2D = new Field2d();

  /** Creates a new DriveSubsystem. */
  public SUBDrive() {
    SmartDashboard.putData("Field", field2D);
  }

  @Override
  public void periodic() {
    field2D.setRobotPose(swerveDriveOdometry.getPoseMeters());
    //  Update the odometry in the periodic block
    swerveDriveOdometry.update(
      Rotation2d.fromDegrees(kGyro.getAngle(IMUAxis.kY)),
      new SwerveModulePosition[] {
        kFrontLeft.getPosition(),
        kFrontRight.getPosition(),
        kRearLeft.getPosition(),
        kRearRight.getPosition()
      }
    );
  }

  /**
  * Returns the currently-estimated pose of the robot.
  *
  * @return The pose.
  */
  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  /**
  * Resets the odometry to the specified pose.
  *
  * @param pose The pose to which to set the odometry.
  */
  public void resetOdometry(Pose2d pose) {
    swerveDriveOdometry.resetPosition(
      Rotation2d.fromDegrees(kGyro.getAngle(IMUAxis.kY)),
      new SwerveModulePosition[] {
        kFrontLeft.getPosition(),
        kFrontRight.getPosition(),
        kRearLeft.getPosition(),
        kRearRight.getPosition()
      },
      pose
    );
  }

  /**
  * Method to drive the robot using joystick info.
  *
  * @param xSpeed        Speed of the robot in the x direction (forward).
  * @param ySpeed        Speed of the robot in the y direction (sideways).
  * @param rot           Angular rate of the robot.
  * @param fieldRelative Whether the provided x and y speeds are relative to the
  *                      field.
  * @param rateLimit     Whether to enable rate limiting for smoother control.
  */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) {// some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        }
        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;
    
      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    //Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, swerveDriveOdometry.getPoseMeters().getRotation())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
    );
    kFrontLeft.setDesiredState(swerveModuleStates[0]);
    kFrontRight.setDesiredState(swerveModuleStates[1]);
    kRearLeft.setDesiredState(swerveModuleStates[2]);
    kRearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
  * Sets the wheels into an X formation to prevent movement.
  */
  public void setX() {
    kFrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    kFrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    kRearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    kRearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));

  }

  /**
  * Sets the swerve ModuleStates.
  *
  * @param desiredStates The desired SwerveModule states.
  */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
    );
    kFrontLeft.setDesiredState(desiredStates[0]);
    kFrontRight.setDesiredState(desiredStates[1]);
    kRearLeft.setDesiredState(desiredStates[2]);
    kRearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    kFrontLeft.resetEncoders();
    kRearLeft.resetEncoders();
    kFrontRight.resetEncoders();
    kRearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    kGyro.reset();
  }

  /**
  * Returns the heading of the robot.
  *
  * @return the robot's heading in degrees, from -180 to 180
  */
  public double getHeading() {
    return swerveDriveOdometry.getPoseMeters().getRotation().getDegrees();
  }

  public void resetGyro() {
    kGyro.reset();
  }

  public ChassisSpeeds getspeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[] {
      kFrontLeft.getState(),
      kFrontRight.getState(),
      kRearLeft.getState(),
      kRearRight.getState()
    };

    return moduleStates;
  }
  
  public SwerveModulePosition[] getPosition() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
      kFrontLeft.getPosition(),
      kFrontRight.getPosition(),
      kRearLeft.getPosition(),
      kRearRight.getPosition()
    };

    return modulePositions;
  }

  /**
  * Returns the turn rate of the robot.
  *
  * @return The turn rate of the robot, in degrees per second
  */
  public double getTurnRate() {
    return kGyro.getRate(IMUAxis.kY) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
    );
    setModuleStates(swerveModuleStates);
  }
}
