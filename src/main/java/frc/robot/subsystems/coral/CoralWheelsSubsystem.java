package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralWheelsSubsystem extends SubsystemBase {

    private final SparkMax wheelMotor;

    public CoralWheelsSubsystem() {

        wheelMotor = new SparkMax(CoralConstants.wheelMotorID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(10);

        wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void stopWheelMotors() {
        wheelMotor.stopMotor();
    }

    public void setWheelMotors(double speed) {
        wheelMotor.set(speed);
    }
    
}
