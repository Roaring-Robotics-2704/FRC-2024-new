package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HookConstants;

public class SUBClimb extends SubsystemBase {
    private final CANSparkMax kLeftHook;
    private final CANSparkMax kRightHook;
    
    private final AbsoluteEncoder kLeftHookEncoder;
    private final AbsoluteEncoder kRightHookEncoder;

    private final SparkPIDController kLeftHookPIDController;
    private final SparkPIDController kRightHookPIDController;

    double leftHookSetPoint = 0;
    double rightHookSetPoint = 0;
   


    public SUBClimb(){
        kLeftHook = new CANSparkMax(HookConstants.kLeftHookCanId,MotorType.kBrushless);
        kRightHook = new CANSparkMax(HookConstants.kRightHookCanId, MotorType.kBrushless);
        kLeftHook.restoreFactoryDefaults();
        kRightHook.restoreFactoryDefaults();

        kLeftHookEncoder = kLeftHook.getAbsoluteEncoder(Type.kDutyCycle);
        kRightHookEncoder = kRightHook.getAbsoluteEncoder(Type.kDutyCycle);
        kLeftHookPIDController = kLeftHook.getPIDController();
        kRightHookPIDController = kRightHook.getPIDController();
        kLeftHookPIDController.setFeedbackDevice(kLeftHookEncoder);
        kRightHookPIDController.setFeedbackDevice(kRightHookEncoder);

        kLeftHookEncoder.setInverted(HookConstants.kLeftHookEncoder);
        kRightHookEncoder.setInverted(HookConstants.kRightHookEncoder);

        kLeftHookPIDController.setPositionPIDWrappingEnabled(true);
        kLeftHookPIDController.setPositionPIDWrappingMinInput(HookConstants.kLeftHookEncoderPositionPIDMinInput);
        kLeftHookPIDController.setPositionPIDWrappingMaxInput(HookConstants.kLeftHookEncoderPositionPIDMaxInput);
        kRightHookPIDController.setPositionPIDWrappingEnabled(true);
        kRightHookPIDController.setPositionPIDWrappingMinInput(HookConstants.kRightHookEncoderPositionPIDMinInput);
        kRightHookPIDController.setPositionPIDWrappingMaxInput(HookConstants.kRightHookEncoderPosiionPIDMaxInput);

        kLeftHookPIDController.setP(HookConstants.kLeftHookEncoderP);
        kLeftHookPIDController.setI(HookConstants.kLeftHookEncoderI);
        kLeftHookPIDController.setD(HookConstants.kLeftHookEncoderD);
        kLeftHookPIDController.setFF(HookConstants.kLeftHookEncoderFF);
        kLeftHookPIDController.setOutputRange(HookConstants.kLeftHookEncoderMinOutput, HookConstants.kLeftHookEncoderMaxOutput);

        kRightHookPIDController.setP(HookConstants.kRightHookEncoderP);
        kRightHookPIDController.setI(HookConstants.kRightHookEncoderI);
        kRightHookPIDController.setD(HookConstants.kRightHookEncoderD);
        kRightHookPIDController.setFF(HookConstants.kRightHookEncoderFF);
        kRightHookPIDController.setOutputRange(HookConstants.kRightHookEncoderMinOutput, HookConstants.kRightHookEncoderMaxOutput);

        kLeftHook.setIdleMode(HookConstants.kLeftHookEncoderIdleMode);
        kRightHook.setIdleMode(HookConstants.kRightHookEncoderIdleMode);
        kLeftHook.setSmartCurrentLimit(HookConstants.kLeftHookEncoderCurrentLimit);
        kRightHook.setSmartCurrentLimit(HookConstants.kRightHookEncoderCurrentLimit);
        kRightHook.setInverted(false);
        kLeftHook.setInverted(true);
        kLeftHook.burnFlash();
        kRightHook.burnFlash();

    }

    public void setLeftHookPosition(double leftHookPosition) {
        leftHookSetPoint = leftHookPosition;
        //kLeftHook.set(leftHookPosition);
    }

    public void setRightHookPosition(double rightHookPosition) {
        rightHookSetPoint = rightHookPosition;
        //kRightHook.set(rightHookPosition);
    }

        public void changeLeftHookPosition(double leftHookPosition) {
        leftHookSetPoint = leftHookPosition + leftHookSetPoint;
        kLeftHook.set(leftHookPosition);
    }

    public void changeRightHookPosition(double rightHookPosition) {
        rightHookSetPoint = rightHookPosition + rightHookSetPoint;
        kRightHook.set(rightHookPosition);
    }

    public double getLeftHookPosition() {
        return kLeftHookEncoder.getPosition();
    }

    public double getRightHookPosition() {
        return kRightHookEncoder.getPosition();
    }


    public void raiseClimber() {

    }


    @Override
    public void periodic() {
        SmartDashboard.setDefaultNumber("hookP", Constants.HookConstants.kP);
        SmartDashboard.setDefaultNumber("hookI", Constants.HookConstants.kI);
        SmartDashboard.setDefaultNumber("hookD", Constants.HookConstants.kD);
        //pid.setP(SmartDashboard.getNumber("hookP", 0));
        //pid.setI(SmartDashboard.getNumber("hookI", 0));
        //pid.setD(SmartDashboard.getNumber("hookD", 0));
        //kRightHookPIDController.setReference(rightHookSetPoint, ControlType.kPosition);
        //kLeftHookPIDController.setReference(leftHookSetPoint, ControlType.kPosition);


    }



}
