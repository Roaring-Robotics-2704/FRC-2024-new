// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
<<<<<<< Updated upstream
import com.pathplanner.lib.path.PathPlannerPath;
=======
import com.pathplanner.lib.path.PathConstraints;
>>>>>>> Stashed changes
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
<<<<<<< Updated upstream
 import frc.robot.commands.CMDAlign;
 import frc.robot.commands.CMDDrive;
import frc.robot.commands.CMDShooter;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
 import frc.robot.subsystems.DriveSubsystem;
=======
import frc.robot.commands.CMDArm;
import frc.robot.commands.CMDClimb;
import frc.robot.commands.CMDDrive;
import frc.robot.commands.CMDShooter;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
>>>>>>> Stashed changes
import frc.robot.subsystems.SUBArm;
import frc.robot.subsystems.SUBShooter;
import frc.robot.subsystems.SUBVision;
import frc.robot.subsystems.SUBShooter.*;
import frc.utils.RoaringUtils;
import frc.utils.RoaringUtils.DeadzoneUtils;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
<<<<<<< Updated upstream
   //The robot's subsystems
   public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
     public static final CMDDrive driveRobotCommand = new CMDDrive();
    public static final SUBShooter m_SUBShooter = new SUBShooter();
    public static final CMDShooter m_CMDShooter = new CMDShooter();
    public static final SUBVision m_SUBVision = new SUBVision();
     public static final CMDAlign m_CMDAlign = new CMDAlign();
    public static final SUBArm m_SUBArm = new SUBArm();






=======
  //The robot's subsystems
  private static final SUBDrive kRobotDrive = new SUBDrive();
  private static final CMDDrive kDriveRobotCommand = new CMDDrive(kRobotDrive);
  private static final SUBShooter kSUBShooter = new SUBShooter();
  private static final CMDShooter kCMDShooter = new CMDShooter(kSUBShooter);
  private static final SUBVision kSUBVision = new SUBVision();
  private static final SUBArm kSUBArm = new SUBArm();
  private static final CMDArm kCMDArm = new CMDArm(kSUBArm);
  private static final SUBClimb kSUBClimb = new SUBClimb();
  private static final CMDClimb kCMDClimb = new CMDClimb(kSUBClimb);

  private static final PathConstraints kPathconstraints = new PathConstraints(5, 3, 360, 15);
  private final SUBPoseEstimator kPoseEstimator = new SUBPoseEstimator( kRobotDrive,kSUBVision);
  public enum RobotMode {
    KitBot, CompBot
  }
  public enum ControlMode {
    Drone, Game
  }
  private static SendableChooser<Boolean> fieldOrientedChooser = new SendableChooser<Boolean>();
  private static SendableChooser<ControlMode> controlChooser = new SendableChooser<ControlMode>();
  private static SendableChooser<Boolean> rateLimitChooser = new SendableChooser<Boolean>();
  private static SendableChooser<RobotMode> robotChooser = new SendableChooser<RobotMode>();
  private static SendableChooser<Command> autoChooser;
>>>>>>> Stashed changes
  
    public static SendableChooser<Boolean> fieldOrientedChooser = new SendableChooser<Boolean>();
    public static SendableChooser<String> controlChooser = new SendableChooser<String>();

  
    public static SendableChooser<Boolean> rateLimitChooser = new SendableChooser<Boolean>();

   //The driver's controller
  public static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public static CommandXboxController m_driverController2 = new CommandXboxController(OIConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    AutoBuilder.configureHolonomic(
            m_robotDrive::getPose,  //Robot pose supplier
                 m_robotDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                 m_robotDrive::getspeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                 m_robotDrive::driveRobotRelative,//  Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(12, 0.0, 0.0),//  Rotation PID constants
                   3,//  Max module speed, in m/s
                         Units.inchesToMeters(18.2), // Drive base radius in meters. Distance from robot center to furthest module.
                         new ReplanningConfig() // Default path replanning config. See the API for the options here
                 ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            m_robotDrive // Reference to this subsystem to set requirements
    );


<<<<<<< Updated upstream
     // Configure the button bindings
=======
     NamedCommands.registerCommand("Take Note", kSUBShooter.getIntakeCommand().withTimeout(1));
     NamedCommands.registerCommand("Shoot Note", kSUBShooter.getIdleCommand()
      .withTimeout(LauncherConstants.kLauncherDelay)
      .andThen(kSUBShooter.getLaunchCommand().withTimeout(LauncherConstants.kLauncherDelay)));
>>>>>>> Stashed changes

            NamedCommands.registerCommand("Intake", m_SUBShooter.getIntakeCommand().withTimeout(1));
 
    fieldOrientedChooser.setDefaultOption("Field Oriented", true);
    fieldOrientedChooser.addOption("Robot Oriented", false);

    rateLimitChooser.setDefaultOption("False", false);
    rateLimitChooser.addOption("True", true);
    controlChooser.setDefaultOption("Drone", "drone");
    controlChooser.addOption("Game", "game");

    SmartDashboard.putData("Rate limit",rateLimitChooser);
    SmartDashboard.putData("Field oriented",fieldOrientedChooser);
    SmartDashboard.putData("Controls", controlChooser);
<<<<<<< Updated upstream
=======
    SmartDashboard.putData("Robot Select", robotChooser);
    
>>>>>>> Stashed changes

    configureButtonBindings();
   //SendableChooser<Command> autoPathChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Path follower", autoPathChooser);
    // Configure default commands
<<<<<<< Updated upstream
    m_SUBShooter.setDefaultCommand(m_CMDShooter);
     m_robotDrive.setDefaultCommand(
       //   The left stick controls translation of the robot.
       //   Turning is controlled by the X axis of the right stick.
         /* 
         new RunCommand(
             () -> m_robotDrive.drive(
                 -DeadzoneUtils.LinearDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                 -DeadzoneUtils.LinearDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                 -DeadzoneUtils.LinearDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                 true, true),
             m_robotDrive));*/
             driveRobotCommand);
            
=======
    kSUBShooter.setDefaultCommand(kCMDShooter);
    kRobotDrive.setDefaultCommand(kDriveRobotCommand);
    kSUBArm.setDefaultCommand(kCMDArm);
    kPoseEstimator.register();
    kSUBVision.register();
    kSUBVision.periodic();
    kPoseEstimator.periodic();
    kSUBClimb.setDefaultCommand(kCMDClimb);


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("auto", autoChooser);
    
    configureButtonBindings();
>>>>>>> Stashed changes
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
         m_driverController.x()
         .whileTrue(new RunCommand(
             () -> m_robotDrive.setX(),
             m_robotDrive));

<<<<<<< Updated upstream


           m_driverController
        .rightBumper()
        .whileTrue(
            new PrepareLaunch(m_SUBShooter)
                .withTimeout(LauncherConstants.kLauncherDelay)
                .andThen(new LaunchNote(m_SUBShooter))
                .handleInterrupt(() -> m_SUBShooter.stop()));

    m_driverController.leftBumper().whileTrue(m_SUBShooter.getIntakeCommand());

 m_driverController.rightStick().whileTrue(m_CMDAlign);
//m_driverController2.y().onTrue(new RunCommand(()-> m_SUBArm.setPosition(ArmConstants.kRaisedPosition), m_SUBArm));
//m_driverController2.a().onTrue(new RunCommand(()-> m_SUBArm.setPosition(ArmConstants.kLowerPosition), m_SUBArm));

=======
    //OIDriverController1.rightTrigger(0.1)
    //  .whileTrue(AutoBuilder.pathfindToPose(new Pose2d(1.75,5.5,Rotation2d.fromDegrees(180)), kPathconstraints));
    OIDriverController2.y().onTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kAmpPosition), kSUBArm));
    OIDriverController2.a().onTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kIntakeUpPosition), kSUBArm).withTimeout(0.5).andThen(()-> kSUBArm.setPosition(ArmConstants.kIntakePosition)));
    //OIDriverController2.b().whileTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kSpeakerPosition),kSUBArm).withTimeout(1).andThen(kSUBShooter.getLaunchCommand()).withTimeout(1));
    OIDriverController2.x().onTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kHoldPosition), kSUBArm));
    OIDriverController2.b().onTrue(new RunCommand(()-> kSUBArm.setPosition(ArmConstants.kSpeakerPosition), kSUBArm));

    // OIDriverController1.leftBumper().whileTrue(new RunCommand(()-> kSUBClimb.setLeftHookPosition(0.1), kSUBClimb));
    // OIDriverController1.leftTrigger().whileTrue(new RunCommand(()-> kSUBClimb.setLeftHookPosition(-0.1), kSUBClimb));
    // OIDriverController1.rightBumper().whileTrue(new RunCommand(()-> kSUBClimb.setRightHookPosition(0.1), kSUBClimb));
    // OIDriverController1.rightTrigger().whileTrue(new RunCommand(()-> kSUBClimb.setRightHookPosition(-0.1), kSUBClimb));
>>>>>>> Stashed changes
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
<<<<<<< Updated upstream
        m_robotDrive.resetOdometry(PathPlannerPath.fromPathFile("Line").getPreviewStartingHolonomicPose());
      return AutoBuilder.buildAuto("Line");
    }
=======
    return autoChooser.getSelected();
  }

  public static CommandXboxController getDriverController1() {
    return new CommandXboxController(OIConstants.kDriverControllerPort);
  }

  public static CommandXboxController getDriverController2() {
    return new CommandXboxController(OIConstants.kDriverControllerPort2);
  }

  public static RobotMode getRobotMode() {
    return robotChooser.getSelected();
  }

  public static ControlMode getControlMode() {
    return controlChooser.getSelected();
  }

  public static boolean isFieldOriented() {
    return fieldOrientedChooser.getSelected();
  }

  public static boolean isRateLimited() {
    return rateLimitChooser.getSelected();
  }

  public static SUBDrive getDriveSubsystem() {
    return kRobotDrive;
  }
>>>>>>> Stashed changes
}