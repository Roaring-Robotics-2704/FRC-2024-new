// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.CMDDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SUBIntake;

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
    public static final SUBIntake m_subIntake = new SUBIntake(500);
    
  
    public static SendableChooser<Boolean> fieldOrientedChooser = new SendableChooser<Boolean>();
  
    public static SendableChooser<Boolean> rateLimitChooser = new SendableChooser<Boolean>();

  // The driver's controller
  static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
      CommandXboxController OIDriver2Controller = new CommandXboxController(OIConstants.kDriver2ControllerPort);


    public final static SUBIntake kSUBIntake = new SUBIntake(ModuleConstants.kIntakeCanId);

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
   //SendableChooser<Command> autoPathChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Path follower", autoPathChooser);
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
    m_driverController.x()
        .whileTrue(new RunCommand(
            m_robotDrive::setX,
            m_robotDrive));
      m_driverController.leftBumper().whileTrue(new RunCommand(()->m_subIntake.setIntakeController(-0.1), m_subIntake));
      m_driverController.leftBumper().whileFalse(new RunCommand(()->m_subIntake.setIntakeController(0), m_subIntake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return AutoBuilder.buildAuto("321");
    }
}