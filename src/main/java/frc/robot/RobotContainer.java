// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CMDAuto;
import frc.robot.commands.Arm.CMDArm;
import frc.robot.commands.Arm.CMDIntake;
import frc.robot.commands.Arm.CMDShoot;
import frc.robot.commands.Climb.CMDClimb;
import frc.robot.commands.Drive.CMDDrive;
import frc.robot.subsystems.Arm.SUBArm;
import frc.robot.subsystems.Arm.SUBIntake;
import frc.robot.subsystems.Arm.SUBShooter;
import frc.robot.subsystems.Drive.SUBSwerveDrive;
import frc.robot.subsystems.Drive.SUBSwerveModule;
import frc.robot.subsystems.Hook.SUBHook;
import frc.robot.subsystems.Vision.SUBPosseEstimation;
import frc.robot.subsystems.Vision.SUBVision;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// The robot's subsystems
    public final SUBHook m_sUBHook = new SUBHook();
    public final SUBShooter m_sUBShooter = new SUBShooter();
    public final SUBIntake m_sUBIntake = new SUBIntake();
    public final SUBArm m_sUBArm = new SUBArm();
    public  static final SUBVision m_SUBVision = new SUBVision();
    public final SUBPosseEstimation m_PosseEstimation = new SUBPosseEstimation(null, null);

    public static final SUBSwerveDrive robotDrive = new SUBSwerveDrive();
    public static final CMDDrive driveRobotCommand = new CMDDrive();
    SendableChooser<String> PathPlannerautoChooser = new SendableChooser<String>();
    SendableChooser<String> ChoreoautoChooser = new SendableChooser<String>();
  
    public static SendableChooser<Boolean> fieldOrientedChooser = new SendableChooser<Boolean>();
    public static SendableChooser<String> pathChooser = new SendableChooser<String>();
  
    public static SendableChooser<Boolean> rateLimitChooser = new SendableChooser<Boolean>();
// Joysticks
private final CommandXboxController OIDriverController2 = new CommandXboxController(1);
private final CommandXboxController OIDriverController1 = new CommandXboxController(0);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {
AutoBuilder.configureHolonomic(
                robotDrive::getPose, // Robot pose supplier
                robotDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                robotDrive::getspeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                robotDrive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
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
                robotDrive // Reference to this subsystem to set requirements
        );

<<<<<<< Updated upstream
    fieldOrientedChooser.setDefaultOption("Field Oriented", true);
=======
    AutoBuilder.configureHolonomic(
      kPoseEstimator::getCurrentPose,  //Robot pose supplier
      kPoseEstimator::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
      kRobotDrive::getspeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      kRobotDrive::driveRobotRelative,//  Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(10, 0.0, 0.0), // Translation PID constants
        new PIDConstants(10, 0.0, 0.0),//  Rotation PID constants
        2,//  Max module speed, in m/s
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
      kRobotDrive // Reference to this subsystem to set requirements
    );
   // Configure the button bindings

    NamedCommands.registerCommand("Take Note", kSUBShooter.getIntakeCommand().repeatedly());
    NamedCommands.registerCommand("Amp Note", new RunCommand(()->kSUBShooter.setWheels(0.5,0.1), kSUBShooter).repeatedly().withTimeout(1.25));//.andThen(new RunCommand(()->kSUBShooter.setWheels(0.0,0.0), kSUBShooter)).withTimeout(0.1));
    NamedCommands.registerCommand("Arm Intake", new RunCommand(()->kSUBArm.setPosition(ArmConstants.kIntakePosition), kSUBArm).repeatedly().withTimeout(1));//.until(()->kSUBArm.isAtSetpoint()));
    NamedCommands.registerCommand("Arm Amp", new RunCommand(()->kSUBArm.setPosition(ArmConstants.kAmpPosition), kSUBArm).repeatedly().withTimeout(1));//.until(()->kSUBArm.isAtSetpoint()));
    NamedCommands.registerCommand("Arm Speaker", new RunCommand(()->kSUBArm.setPosition(ArmConstants.kSpeakerPosition+Units.degreesToRadians(0)), kSUBArm).withTimeout(1));
    NamedCommands.registerCommand("Speaker Note", new RunCommand(()->kSUBShooter.setWheels(0.0,1.0),kSUBShooter).repeatedly().withTimeout(3).andThen(new RunCommand(()->kSUBShooter.setWheels(0.6,1.0), kSUBShooter).repeatedly().withTimeout(1)));
      // ()->kSUBShooter.setLaunchWheel(1), kSUBShooter).repeatedly().withTimeout(3)
      // .andThen(new RunCommand(()->kSUBShooter.setWheels(0.6,1.0),kSUBShooter).repeatedly().withTimeout(1)));
    fieldOrientedChooser.setDefaultOption("Field Oriented", true);  
    NamedCommands.registerCommand("Bring Note Back", new RunCommand(()->kSUBShooter.setWheels(-0.1,-0.1), kSUBShooter).repeatedly().alongWith(new RunCommand(()->kSUBArm.setPosition(ArmConstants.kIntakeUpPosition), kSUBArm)).withTimeout(0.75).andThen(new RunCommand(()->kSUBShooter.setWheels(-0.0,-0.0), kSUBShooter).withTimeout(0.1)));
>>>>>>> Stashed changes
    fieldOrientedChooser.addOption("Robot Oriented", false);

    rateLimitChooser.setDefaultOption("False", false);
    rateLimitChooser.addOption("True", true);

    SmartDashboard.putData(rateLimitChooser);
    SmartDashboard.putData(fieldOrientedChooser);

    configureButtonBindings();
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
    SmartDashboard.putData("Choreo path Chooser",ChoreoautoChooser);
    SmartDashboard.putData("Path planner chooser", PathPlannerautoChooser);
    pathChooser.setDefaultOption("Choreo", "choreo");
    pathChooser.addOption("PathPlanner", "pathplanner");
    pathChooser.addOption("No auto", "none");
    SmartDashboard.putData("path follow chooser", pathChooser);

    

    robotDrive.setDefaultCommand(driveRobotCommand);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    OIDriverController1.x().whileTrue(new RunCommand(() -> robotDrive.setX(), robotDrive));

}

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public CommandXboxController getOIDriverController1() {
      return OIDriverController1;
    }

public CommandXboxController getOIDriverController2() {
      return OIDriverController2;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
      if (pathChooser.getSelected() == "choreo") {
      PathPlannerPath ChoreoTraj = PathPlannerPath.fromChoreoTrajectory(ChoreoautoChooser.getSelected());
      return AutoBuilder.followPath(ChoreoTraj);
    } else if (pathChooser.getSelected() == "pathplanner") {
      return AutoBuilder.buildAuto(PathPlannerautoChooser.getSelected());
    }
    else return null;
  }
  

}

