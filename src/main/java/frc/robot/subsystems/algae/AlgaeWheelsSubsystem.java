package frc.robot.subsystems.algae;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeWheelsSubsystem extends SubsystemBase {

    private final SparkMax rightMotor;
    private final SparkMax leftMotor;

    public AlgaeWheelsSubsystem() {

        rightMotor = new SparkMax(AlgaeConstants.rightMotorID, MotorType.kBrushless);
        leftMotor = new SparkMax(AlgaeConstants.leftMotorID, MotorType.kBrushless);
        
    }

    public void stopWheelMotors() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public void setWheelMotors(double speed) {
        rightMotor.set(speed);
        leftMotor.set(-speed);
    }
    
}
