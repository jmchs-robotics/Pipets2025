package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;

public class AlgaeSubsystem extends SubsystemBase {

    private final SparkMax rightMotor;
    private final SparkMax leftMotor;
    private final TalonFX flipMotor;
    private final PIDController pidController;

    public AlgaeSubsystem() {

        rightMotor = new SparkMax(AlgaeConstants.rightMotorID, MotorType.kBrushless);
        leftMotor = new SparkMax(AlgaeConstants.leftMotorID, MotorType.kBrushless);
        flipMotor = new TalonFX(AlgaeConstants.flipMotorID);
        
        // constraints = new TrapezoidProfile.Constraints(
        //     AlgaeConstants.maxVelocity,
        //     AlgaeConstants.maxAcceleration
        // );
        
        pidController = new PIDController(
            AlgaeConstants.kP,
            AlgaeConstants.kI,
            AlgaeConstants.kD
        );
        
        pidController.setTolerance(0.1); // 0.5 inches position tolerance
        
        // Initialize states and profile
        // currentState = new TrapezoidProfile.State(0, 0);
        // goalState = new TrapezoidProfile.State(0, 0);
        // profile = new TrapezoidProfile(constraints);

        flipMotor.setPosition(0);

    }

    public void stopWheelMotors() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public void stopFlipperMotor() {
        flipMotor.stopMotor();
    }

    public void setWheelMotors(double speed) {
        rightMotor.set(speed);
        leftMotor.set(-speed);
    }

    public void flipFlipperUp() {
        double rawOutput = pidController.calculate(flipMotor.getPosition().getValueAsDouble(), -3);
        double output = MathUtil.clamp(rawOutput, -1, 1);
        flipMotor.set(output);
    }

    public void flipFlipperDown() {
        double rawOutput = pidController.calculate(flipMotor.getPosition().getValueAsDouble(), 0);
        double output = MathUtil.clamp(rawOutput, -1, 1);
        flipMotor.set(output);
    }
}
