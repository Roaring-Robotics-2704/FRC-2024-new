// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CMDDrive;
import frc.robot.commands.LaunchNote;
import frc.robot.commands.PrepareLaunch;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SUBShooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
    public static final CMDDrive driveRobotCommand = new CMDDrive();
    public static final SUBShooter m_sUBShooter = new SUBShooter();

SendableChooser<String> PathPlannerautoChooser = new SendableChooser<String>();
    SendableChooser<String> ChoreoautoChooser = new SendableChooser<String>();
  
    public static SendableChooser<Boolean> fieldOrientedChooser = new SendableChooser<Boolean>();
    public static SendableChooser<String> pathChooser = new SendableChooser<String>();
  
    public static SendableChooser<Boolean> rateLimitChooser = new SendableChooser<Boolean>();

  // The driver's controller
  public static CommandXboxController m_driverController1 = new CommandXboxController(OIConstants.kDriverControllerPort1);
  public static CommandXboxController m_driverController2 = new CommandXboxController(OIConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    AutoBuilder.configureHolonomic(
                m_robotDrive::getPose, // Robot pose supplier
                m_robotDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                m_robotDrive::getspeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                m_robotDrive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(12, 0.0, 0.0), // Rotation PID constants
                        3, // Max module speed, in m/s
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


    fieldOrientedChooser.setDefaultOption("Field Oriented", true);
    fieldOrientedChooser.addOption("Robot Oriented", false);

    rateLimitChooser.setDefaultOption("False", false);
    rateLimitChooser.addOption("True", true);

    SmartDashboard.putData("Rate limit",rateLimitChooser);
    SmartDashboard.putData("Field oriented",fieldOrientedChooser);

    configureButtonBindings();
    try {
   for (String option : AutoBuilder.getAllAutoNames()) {
      // Assuming {String here} represents the same string, you can modify this part as needed
      PathPlannerautoChooser.addOption(option.toString(), option.toString());
  }
    File deploy = Filesystem.getDeployDirectory();
File pathfolder = new File(Path.of(deploy.getAbsolutePath(),"choreo").toString());
File[] listOfFiles = pathfolder.listFiles();

for (int i = 0; i < listOfFiles.length; i++) {
  if (listOfFiles[i].isFile()) {
    System.out.println("path:" + listOfFiles[i].getName());
    ChoreoautoChooser.addOption(listOfFiles[i].getName().replace(".traj", ""), listOfFiles[i].getName().replace(".traj", ""));
  }
}
    } finally {}
    SmartDashboard.putData("Choreo path Chooser",ChoreoautoChooser);
    SmartDashboard.putData("Path planner chooser", PathPlannerautoChooser);
    pathChooser.setDefaultOption("Choreo", "choreo");
    pathChooser.addOption("PathPlanner", "pathplanner");
    pathChooser.addOption("No auto", "none");
    SmartDashboard.putData("Path follower", pathChooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        /*new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
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
    m_driverController1.x()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    m_driverController2.rightTrigger()
        .whileTrue(
            new PrepareLaunch(m_sUBShooter)
                .withTimeout(LauncherConstants.kLauncherDelay)
                .andThen(new LaunchNote(m_sUBShooter))
                .handleInterrupt(() -> m_sUBShooter.stop()));

    m_driverController2.leftTrigger()
        .whileTrue(m_sUBShooter.getIntakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
          if (pathChooser.getSelected() == "choreo") {
      PathPlannerPath ChoreoTraj = PathPlannerPath.fromChoreoTrajectory(ChoreoautoChooser.getSelected());
      return AutoBuilder.followPath(ChoreoTraj);
    } else if (pathChooser.getSelected() == "pathplanner") {
      return AutoBuilder.buildAuto(PathPlannerautoChooser.getSelected());
    }
    else return null;
  }
}