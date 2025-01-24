package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX primaryMotor;
    private final TalonFX followerMotor;
    private final PIDController pidController;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;
    private final TrapezoidProfile profile;

    private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
    private boolean isHomed = false;
    private double setpoint = 0.0;
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    double currentPos;

    public enum ElevatorPosition {
        DOWN(0),
        POSITION_1(1),
        POSITION_2(2),
        POSITION_3(3),
        POSITION_4(4);

        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }

    public ElevatorSubsystem() {
        primaryMotor = new TalonFX(1);
        followerMotor = new TalonFX(2);
        
        primaryMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);

        followerMotor.setControl(new Follower(1, true));

        constraints = new TrapezoidProfile.Constraints(
            2,
            2
        );
        
        pidController = new PIDController(
            0.05,
            0,
            0
        );
        
        pidController.setTolerance(0.5); // 0.5 inches position tolerance
        
        // Initialize states and profile
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);

    }

    @Override
    public void periodic() {

        // currentPos has been converted to rotations
        // Clicks / (Clicks / Rotations) = Rotations
        // TODO: Figure out how many inches one rotation of the motor will raise the elevator
        currentPos = primaryMotor.getPosition().getValueAsDouble() / 2048;
        
        // Calculate the next state and update current state
        currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

        if (getHeightInches() > 74) {
            stopMotors();
        }

        // Only run control if homed
        // if (isHomed) {
        //     double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
        //     double ff = calculateFeedForward(currentState);
            
        //     double outputPower = MathUtil.clamp(
        //         pidOutput + ff,
        //         -ElevatorConstants.max_output,
        //         ElevatorConstants.max_output
        //     );
            
        //     primaryMotor.set(outputPower);
        // }
    }

    private void handleBottomLimit() {
        stopMotors();
        primaryMotor.setPosition(0);
        isHomed = true;
        setpoint = 0;
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        pidController.reset();
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    public boolean isAtHeight(double targetHeightInches) {
        // Check if the elevator is within a small tolerance of the target height
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - targetHeightInches) < 0.1;
    }
    

    // TODO: Gotta figure out kS and kG and kV
    private double calculateFeedForward(TrapezoidProfile.State state) {
        // kS (static friction), kG (gravity), kV (velocity),
        // return ElevatorConstants.kS * Math.signum(state.velocity) +
        //        ElevatorConstants.kG +
        //        ElevatorConstants.kV * state.velocity;

        return 0.0;
    }

    public void setPositionInches(double inches) {
        if (!isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        setpoint = MathUtil.clamp(
            inches,
            0,
            74
        );
        
        // Update goal state for motion profile
        goalState = new TrapezoidProfile.State(setpoint, 0);
    }

    // TODO: Figure out how many inches one rotation is
    public double getHeightInches() {
        return primaryMotor.getPosition().getValueAsDouble() / 2048;
    }

    public void homeElevator() {
        primaryMotor.set(-0.1); // Slow downward movement until bottom limit is hit
        // if (bottomLimit.get()) {
        //     handleBottomLimit();
        // }
    }

    public boolean isAtPosition(ElevatorPosition position) {
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - position.positionInches) < 0.5;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public ElevatorPosition getCurrentTarget() {
        return currentTarget;
    }

    // public void setManualPower(double power) {
    //     // Disable PID control when in manual mode
    //     pidController.reset();
    //     currentState = new TrapezoidProfile.State(getHeightInches(), 0);
    //     goalState = new TrapezoidProfile.State(getHeightInches(), 0);
        
    //     if (!isHomed && power < 0) {
    //         power = 0;
    //     }
        
    //     if (getHeightInches() >= 74 && power > 0) {
    //         power = 0;
    //     }
        
    //     // if (bottomLimit.get() && power < 0) {
    //     //     power = 0;
    //     // }
        
    //     // primaryMotor.set(MathUtil.clamp(power, -ElevatorConstants.max_output, ElevatorConstants.max_output));
    // }
}