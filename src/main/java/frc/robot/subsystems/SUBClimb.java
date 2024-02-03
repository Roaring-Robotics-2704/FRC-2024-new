package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.AbsoluteEncoder;

public class SUBClimb extends SubsystemBase {
    private final CANSparkMax kLeftHook;
    private final CANSparkMax kRightHook;
    
    private final AbsoluteEncoder kLeftHookEncoder;
    private final AbsoluteEncoder kRightHookEncoder;

    private final SparkPIDController kLeftHookPIDController;
    private final SparkPIDController kRightHookPIDController;

    double leftHookSetPoint = 0;
    double rightHookSetPoint = 0;
   
   PIDController pid = new PIDController(Constants.HookConstants.kP, Constants.HookConstants.kI, Constants.HookConstants.kD);

    public SUBClimb(int LeftHookCANId, int RightHookCANId){
        kLeftHook = new CANSparkMax(LeftHookCANId, MotorType.kBrushless);
        kRightHook = new CANSparkMax(RightHookCANId, MotorType.kBrushless);

        kLeftHook.restoreFactoryDefaults();
        kRightHook.restoreFactoryDefaults();

        kLeftHookEncoder = kLeftHook.getAbsoluteEncoder(Type.kDutyCycle);
        kRightHookEncoder = kRightHook.getAbsoluteEncoder(Type.kDutyCycle);
        kLeftHookPIDController = kLeftHook.getPIDController();
        kRightHookPIDController = kRightHook.getPIDController();
        kLeftHookPIDController.setFeedbackDevice(kLeftHookEncoder);
        kRightHookPIDController.setFeedbackDevice(kRightHookEncoder);

        kLeftHookEncoder.setInverted(ModuleConstants.kLeftHookEncoder);
        kRightHookEncoder.setInverted(ModuleConstants.kRightHookEncoder);

        kLeftHookPIDController.setPositionPIDWrappingEnabled(true);
        kLeftHookPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kLeftHookEncoderPositionPIDMinInput);
        kLeftHookPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kLeftHookEncoderPositionPIDMaxInput);
        kRightHookPIDController.setPositionPIDWrappingEnabled(true);
        kRightHookPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kRightHookEncoderPositionPIDMinInput);
        kRightHookPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kRightHookEncoderPosiionPIDMaxInput);

        kLeftHookPIDController.setP(ModuleConstants.kLeftHookEncoderP);
        kLeftHookPIDController.setI(ModuleConstants.kLeftHookEncoderI);
        kLeftHookPIDController.setD(ModuleConstants.kLeftHookEncoderD);
        kLeftHookPIDController.setFF(ModuleConstants.kLeftHookEncoderFF);
        kLeftHookPIDController.setOutputRange(ModuleConstants.kLeftHookEncoderMinOutput, ModuleConstants.kLeftHookEncoderMaxOutput);

        kRightHookPIDController.setP(ModuleConstants.kRightHookEncoderP);
        kRightHookPIDController.setI(ModuleConstants.kRightHookEncoderI);
        kRightHookPIDController.setD(ModuleConstants.kRightHookEncoderD);
        kRightHookPIDController.setFF(ModuleConstants.kRightHookEncoderFF);
        kRightHookPIDController.setOutputRange(ModuleConstants.kRightHookEncoderMinOutput, ModuleConstants.kRightHookEncoderMaxOutput);

        kLeftHook.setIdleMode(ModuleConstants.kLeftHookEncoderIdleMode);
        kRightHook.setIdleMode(ModuleConstants.kRightHookEncoderIdleMode);
        kLeftHook.setSmartCurrentLimit(ModuleConstants.kLeftHookEncoderCurrentLimit);
        kRightHook.setSmartCurrentLimit(ModuleConstants.kRightHookEncoderCurrentLimit);

        kLeftHook.burnFlash();
        kRightHook.burnFlash();

    }

    public void setLeftHookPosition(double leftHookPosition) {
        leftHookSetPoint = leftHookPosition;
    }
    public void setRightHookPosition(double rightHookPosition) {
        rightHookSetPoint = rightHookPosition;
    }

    @Override
    public void periodic() {
        SmartDashboard.setDefaultNumber("hookP", Constants.HookConstants.kP);
        SmartDashboard.setDefaultNumber("hookI", Constants.HookConstants.kI);
        SmartDashboard.setDefaultNumber("hookD", Constants.HookConstants.kD);
        pid.setP(SmartDashboard.getNumber("hookP", 0));
        pid.setI(SmartDashboard.getNumber("hookI", 0));
        pid.setD(SmartDashboard.getNumber("hookD", 0));
    }
}
