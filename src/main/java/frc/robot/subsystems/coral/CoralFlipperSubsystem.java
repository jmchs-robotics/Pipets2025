package frc.robot.subsystems.coral;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralFlipperSubsystem extends SubsystemBase {
    private final TalonFX flipMotor;
    private final PIDController pidController;

    public CoralFlipperSubsystem() {

        flipMotor = new TalonFX(CoralConstants.flipMotorID);
        
        // constraints = new TrapezoidProfile.Constraints(
        //     AlgaeConstants.maxVelocity,
        //     AlgaeConstants.maxAcceleration
        // );
        
        pidController = new PIDController(
            CoralConstants.kP,
            CoralConstants.kI,
            CoralConstants.kD
        );
        
        pidController.setTolerance(0.1); // 0.5 inches position tolerance
        
        // Initialize states and profile
        // currentState = new TrapezoidProfile.State(0, 0);
        // goalState = new TrapezoidProfile.State(0, 0);
        // profile = new TrapezoidProfile(constraints);

        flipMotor.setPosition(0);

    }

    public void stopFlipperMotor() {
        flipMotor.stopMotor();
    }

    public void flipCoralFlipperUp() {
        double rawOutput = pidController.calculate(flipMotor.getPosition().getValueAsDouble(), -3);
        double output = MathUtil.clamp(rawOutput, -1, 1);
        flipMotor.set(output);
    }

    public void flipCoralFlipperDown() {
        double rawOutput = pidController.calculate(flipMotor.getPosition().getValueAsDouble(), 0);
        double output = MathUtil.clamp(rawOutput, -1, 1);
        flipMotor.set(output);
    }
}
