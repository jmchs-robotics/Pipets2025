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
        
        pidController.setTolerance(0.5); // 0.5 inches position tolerance
        
        // Initialize states and profile
        // currentState = new TrapezoidProfile.State(0, 0);
        // goalState = new TrapezoidProfile.State(0, 0);
        // profile = new TrapezoidProfile(constraints);

        flipMotor.setPosition(0);

    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    public void flipFlipper() {}
        

    public void stopIntakeMotors() {
        rightMotor.set(0);
        leftMotor.set(0);
        pidController.reset();
    }

    public void stopFlipMotor() {
        flipMotor.set(0);
        pidController.reset();
    }

}
