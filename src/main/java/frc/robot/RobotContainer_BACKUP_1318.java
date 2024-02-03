// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
=======
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.File;
import java.nio.file.Path;
import java.util.List;

>>>>>>> ArmRC
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
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
 import frc.robot.commands.CMDAlign;
 import frc.robot.commands.CMDDrive;
import frc.robot.commands.CMDShooter;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
 import frc.robot.subsystems.DriveSubsystem;
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
<<<<<<< HEAD
   //The robot's subsystems
   public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
     public static final CMDDrive driveRobotCommand = new CMDDrive();
    public static final SUBShooter m_SUBShooter = new SUBShooter();
    public static final CMDShooter m_CMDShooter = new CMDShooter();
    public static final SUBVision m_SUBVision = new SUBVision();
     public static final CMDAlign m_CMDAlign = new CMDAlign();
    public static final SUBArm m_SUBArm = new SUBArm();



=======
  // The robot's subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final CMDDrive driveRobotCommand = new CMDDrive();
  public final static SUBArm armSubsystem = new SUBArm();
  public static final CMDArmHigh armCommandHigh = new CMDArmHigh(armSubsystem);
  public static final CMDArmLow armCommandLow = new CMDArmLow(armSubsystem);
>>>>>>> ArmRC



  
    public static SendableChooser<Boolean> fieldOrientedChooser = new SendableChooser<Boolean>();
    public static SendableChooser<String> controlChooser = new SendableChooser<String>();

  
    public static SendableChooser<Boolean> rateLimitChooser = new SendableChooser<Boolean>();

<<<<<<< HEAD
   //The driver's controller
  public static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public static CommandXboxController m_driverController2 = new CommandXboxController(OIConstants.kDriverControllerPort2);
=======
  // The driver's controller
  static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
>>>>>>> ArmRC

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


     // Configure the button bindings

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

    configureButtonBindings();
   //SendableChooser<Command> autoPathChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Path follower", autoPathChooser);
    // Configure default commands
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
<<<<<<< HEAD
         m_driverController.x()
         .whileTrue(new RunCommand(
             () -> m_robotDrive.setX(),
             m_robotDrive));



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
    m_driverController.x()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    m_driverController.a().onTrue(armCommandLow);
    m_driverController.b().onTrue(armCommandHigh);

    
>>>>>>> ArmRC
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        m_robotDrive.resetOdometry(PathPlannerPath.fromPathFile("Line").getPreviewStartingHolonomicPose());
      return AutoBuilder.buildAuto("Line");
    }
}