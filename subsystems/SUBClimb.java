package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class SUBClimb extends SubsystemBase {
    private final CANSparkMax kLeftHook;
    private final CANSparkMax kRightHook;
    
    private final AbsoluteEncoder kLeftEncoder;
    private final AbsoluteEncoder kRightEncoder;

    private final SparkPIDController kLeftPIDController;
    private final SparkPIDController kRightPIDController;

    public SUBClimb(int LeftHookCANId, int RightHookCANId){
        kLeftHook = new CANSparkMax(LeftHookCANId, MotorType.kBrushless);
        kRightHook = new CANSparkMax(RightHookCANId, MotorType.kBrushless);

        kLeftHook.restoreFactoryDefaults();
        kRightHook.restoreFactoryDefaults();

        kLeftEncoder = kLeftHook.getAbsoluteEncoder(Type.kDutyCycle);
        kRightEncoder= kRightHook.getAbsoluteEncoder(Type.kDutyCycle);
        kLeftPIDController = kLeftHook.getPIDController();
        kRightPIDController = kRightHook.getPIDController();
        kLeftPIDController.setFeedbackDevice(kLeftEncoder);
        kRightPIDController.setFeedbackDevice(kRightEncoder);

        kLeftEncoder.setInverted(ModuleConstants.kLeftHookEncoder);
        kRightEncoder.setInverted(ModuleConstants.kRightHookEncoder);

        kLeftPIDController.setPositionPIDWrappingEnabled(true);
        kLeftPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kLeftEncoderPositionPIDMinInput);
        kLeftPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kLeftEncoderPositionPIDMaxInput);
        kRightPIDController.setPositionPIDWrappingEnabled(true);
        kRightPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kRightEncoderPositionPIDMinInput);
        kRightPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kRightEncoderPosiionPIDMaxInput);

        kLeftPIDController.setP(ModuleConstants.kLeftEncoderP);
        kLeftPIDController.setI(ModuleConstants.kLeftEncoderI);
        kLeftPIDController.setD(ModuleConstants.kLeftEncoderD);
        kLeftPIDController.setFF(ModuleConstants.kLeftEncoderFF);
        kLeftPIDController.setOutputRange(ModuleConstants.kLeftEncoderMinOutput, ModuleConstants.kLeftEncoderMaxOutput);

        kRightPIDController.setP(ModuleConstants.kRightEncoderP);
        kRightPIDController.setI(ModuleConstants.kRightEncoderI);
        kRightPIDController.setD(ModuleConstants.kRightEncoderD);
        kRightPIDController.setFF(ModuleConstants.kRightEncoderFF);
        kRightPIDController.setOutputRange(ModuleConstants.kRightEncoderMinOutput, ModuleConstants.kRightEncoderMaxOutput);

        kLeftHook.setIdleMode(ModuleConstants.kLeftEncoderIdleMode);
        kRightHook.setIdleMode(ModuleConstants.kRightEncoderIdleMode);
        kLeftHook.setSmartCurrentLimit(ModuleConstants.kLeftEncoderCurrentLimit);
        kRightHook.setSmartCurrentLimit(ModuleConstants.kRightEncoderCurrentLimit);

        kLeftHook.burnFlash();
        kRightHook.burnFlash();

        
    }
}
