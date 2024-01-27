package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SUBIntake extends SubsystemBase {
    private final CANSparkMax kIntakeController;
    private final RelativeEncoder kIntakeEncoder;
    

    public SUBIntake(int canId) {
        kIntakeController = new CANSparkMax(canId, MotorType.kBrushless);
        kIntakeController.restoreFactoryDefaults();
        kIntakeEncoder = kIntakeController.getEncoder();
        kIntakeEncoder.setInverted(ModuleConstants.kIntakeEncoderInverted);
        kIntakeController.setIdleMode(ModuleConstants.kIntakeControllerIdleMode);
        kIntakeController.setSmartCurrentLimit(ModuleConstants.kIntakeControllerCurrentLimit);
        kIntakeController.burnFlash();

        kIntakeEncoder.setPosition(0);

    }

    public void setIntakeController(double percent) {
        kIntakeController.set(percent);
        SmartDashboard.putNumber("intake power (%)", percent);
    }
}