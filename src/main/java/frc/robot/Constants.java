// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/*
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode currentMode = Mode.REAL;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public static final class DriveConstants {
    private DriveConstants() {

    }

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.6;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 5; // radians per second
    public static final double kMagnitudeSlewRate = 5; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 5; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = Units.degreesToRadians(-90);
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Units.degreesToRadians(180);
    public static final double kBackRightChassisAngularOffset = Units.degreesToRadians(90);
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    private ModuleConstants(){}
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = 
      (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = 
      (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
      ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1.5;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    //setting the driving motor current limit will be an in-season project, for now 50A is an ok default
    public static final int kDrivingMotorCurrentLimit = 45; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class HookConstants {
    public static final int kLeftHookCanId = 14;
    public static final int kRightHookCanId = 15;

    public static final boolean kLeftHookEncoder = false;
    public static final boolean kRightHookEncoder = false;

    public static final double kLeftHookEncoderPositionFactor = (2 * Math.PI);
    public static final double kRightHookEncoderPositionFactor = (2 * Math.PI);

    public static final double kLeftHookEncoderPositionPIDMinInput = 0;
    public static final double kLeftHookEncoderPositionPIDMaxInput = kLeftHookEncoderPositionFactor;
    public static final double kRightHookEncoderPositionPIDMinInput = 0;
    public static final double kRightHookEncoderPosiionPIDMaxInput = kRightHookEncoderPositionFactor;

    public static final double kLeftHookEncoderP = 3.75;
    public static final double kLeftHookEncoderI = 0;
    public static final double kLeftHookEncoderD = 0;
    public static final double kLeftHookEncoderFF = 0;
    public static final double kLeftHookEncoderMinOutput = -1;
    public static final double kLeftHookEncoderMaxOutput = 1;

    public static final double kRightHookEncoderP = 3.75;
    public static final double kRightHookEncoderI = 0;
    public static final double kRightHookEncoderD = 0;
    public static final double kRightHookEncoderFF = 0;
    public static final double kRightHookEncoderMinOutput = -1;
    public static final double kRightHookEncoderMaxOutput = 1;

    public static final IdleMode kLeftHookEncoderIdleMode = IdleMode.kBrake;
    public static final IdleMode kRightHookEncoderIdleMode = IdleMode.kBrake;

    public static final int kLeftHookEncoderCurrentLimit = 45;
    public static final int kRightHookEncoderCurrentLimit = 45;
    
    public static final double kRaisedHookPosition = 0;
    public static final double kLowerHookPosition = 0;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static final class OIConstants {
    private OIConstants(){}
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    private AutoConstants(){}
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kIXController = 0;
    public static final double kIYController = 0;
    public static final double kDXController = 0;
    public static final double kDYController = 0;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
      );
  }

  public static final class NeoMotorConstants {
    private NeoMotorConstants(){}
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final int kstrandPort = 0;
  public static final int kStrandLength = 72;

  //public static int[] kyellowRGB = {255, 150, 0};
  public static int[] kyellowRGB = {64,38,0};
  //public static int[] kblueRGB = {0,100,255};
  public static int[] kblueRGB = {0,25,64};
  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 11;
    public static final int kLauncherID1 = 12;
    public static final int kLauncherID2 = 13;
    public static final int kLauncherCurrentLimit = 30;
    public static final int kFeedCurrentLimit =30;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLauncherSpeed = 0.5;
    public static final double kLaunchFeederSpeed = 0.4;
    public static final double kIntakeLauncherSpeed = -1.5;
    public static final double kIntakeFeederSpeed = -.5;
    public static final double kIntakeDuration = 0.1;
    public static final double kLauncherDelay = 0.5;
    public static final double kP = 0.0001;
    public static final double kPLow = 0.00008;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFTop = 0.0003;
    public static final double kFbottom = 0.000225;

    public static final int RPMReducer = 50;
	  public static final int kShooterRPM = 5000 - RPMReducer;
    public static final int kShooterRPMLow = 4900- RPMReducer;
    public static final double kTolerance = 1;
  }

  public static class ArmConstants {
    public static final int kArmMotor1 = 9;
    public static final int kArmMotor2 = 10;
    public static final double kAmpPosition = Units.degreesToRotations(78);//80 is top
    public static final double kIntakePosition = Units.degreesToRotations(-1);
    public static final double kIntakeUpPosition = kIntakePosition + Units.degreesToRotations(5);
    public static final double kSpeakerPosition = Units.degreesToRotations(20);
    public static final double kInsidePosition = Units.degreesToRotations(65);
    public static final double kHoldPosition = Units.degreesToRotations(6.5);
    public static final double kP = 2.7;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int kMotorCurrentLimit = 30; // amps
    public static final double kIncrementAmount = 0.5;
  }

  public static class VisionConstants {

    public static final String APRILTAG_CAMERA_NAME = "PiCam";

    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
      new Translation3d(Units.inchesToMeters(14),0,Units.inchesToMeters(13.5)),//Units.inchesToMeters(17.5)),
      new Rotation3d(0.0, Units.degreesToRadians(20), Units.degreesToRadians(180))
    );
    
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
    public static final Transform3d ROBOT_TO_CAMERA = APRILTAG_CAMERA_TO_ROBOT.inverse();

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }

  public static class Vision {
    public static final String kCameraName = VisionConstants.APRILTAG_CAMERA_NAME;
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam = VisionConstants.APRILTAG_CAMERA_TO_ROBOT;
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
