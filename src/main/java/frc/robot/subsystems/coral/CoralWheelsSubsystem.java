package frc.robot.subsystems.coral;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralWheelsSubsystem extends SubsystemBase {

    private final SparkMax wheelMotor;

    public CoralWheelsSubsystem() {

        wheelMotor = new SparkMax(CoralConstants.wheelMotorID, MotorType.kBrushless);
        
    }

    public void stopWheelMotors() {
        wheelMotor.stopMotor();
    }

    public void setWheelMotors(double speed) {
        wheelMotor.set(speed);
    }
    
}
