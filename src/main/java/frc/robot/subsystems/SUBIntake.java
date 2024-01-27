package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

public class SUBIntake extends SubsystemBase {
    CANSparkMax intakeController;

    public SUBIntake(int canId) {
        intakeController = new CANSparkMax(canId, CANSparkLowLevel.MotorType.kBrushless);
        intakeController.restoreFactoryDefaults();
    }
}