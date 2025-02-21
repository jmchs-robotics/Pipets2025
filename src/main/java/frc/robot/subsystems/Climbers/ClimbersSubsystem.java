package frc.robot.subsystems.Climbers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimbersSubsystem extends SubsystemBase {

   private final TalonFX climbMotor;
    private final PIDController pidController;

    public ClimbersSubsystem() {

        climbMotor = new TalonFX(ClimberConstants.climbMotorID);
        
        // constraints = new TrapezoidProfile.Constraints(
        //     ClimberConstants.maxVelocity,
        //     ClimberConstants.maxAcceleration
        // );
        
        pidController = new PIDController(
            ClimberConstants.kP,
            ClimberConstants.kI,
            ClimberConstants.kD
        );
        
        pidController.setTolerance(0.1); // 0.5 inches position tolerance
        
        // Initialize states and profile
        // currentState = new TrapezoidProfile.State(0, 0);
        // goalState = new TrapezoidProfile.State(0, 0);
        // profile = new TrapezoidProfile(constraints);

        climbMotor.setPosition(0);

    }

    public void stopClimbMotor() {
        climbMotor.stopMotor();
    }

    public void climbDown() {
        double rawOutput = pidController.calculate(climbMotor.getPosition().getValueAsDouble(), -3);
        double output = MathUtil.clamp(rawOutput, -1, 1);
        climbMotor.set(output);
    }

    public void climbUp() {
        double rawOutput = pidController.calculate(climbMotor.getPosition().getValueAsDouble(), 0);
        double output = MathUtil.clamp(rawOutput, -1, 1);
        climbMotor.set(output);
    }
}