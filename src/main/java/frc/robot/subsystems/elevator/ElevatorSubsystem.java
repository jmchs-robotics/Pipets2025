package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX primaryMotor;
    private final TalonFX followerMotor;
    private final DigitalInput bottomLimit;
    private final PIDController pidController;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;
    private final TrapezoidProfile profile;

    private ElevatorPosition currentTarget = ElevatorPosition.HOMED;
    private boolean isHomed = false;
    private double setpoint = 0.0;
    double currentPos;

    private final GenericEntry[] elevatorLevels = new GenericEntry[4];
    public static final boolean[] levelBooleans = new boolean[4];

    public enum ElevatorPosition {
        HOMED(ElevatorConstants.minPos),
        L2_ALGAE(ElevatorConstants.L2_ALGAE),
        L3_ALGAE(ElevatorConstants.L3_ALGAE);

        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }

    public ElevatorSubsystem() {
        primaryMotor = new TalonFX(ElevatorConstants.primaryElevatorID);
        followerMotor = new TalonFX(ElevatorConstants.followerElevatorID);
        
        followerMotor.setControl(new Follower(ElevatorConstants.primaryElevatorID, true));

        bottomLimit = new DigitalInput(ElevatorConstants.limitSwitchPort);

        constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.maxVelocity,
            ElevatorConstants.maxAcceleration
        );
        
        pidController = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD
        );
        
        pidController.setTolerance(0.5); // 0.5 inches position tolerance
        
        // Initialize states and profile
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);

        primaryMotor.setPosition(0);
        followerMotor.setPosition(0);

        setUpDriverTab();
    }

    @Override
    public void periodic() {

        currentPos = primaryMotor.getPosition().getValueAsDouble() / ElevatorConstants.rotationsPerInch;
        
        // Calculate the next state and update current state
        currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

        // if (!bottomLimit.get()) {
        //     handleBottomLimit();
        // }

        if (getHeightInches() > ElevatorConstants.maxPos) {
            stopMotors();
        }

        // // Only run control if homed
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

        // Update SmartDashboard
        updateTelemetry();
        // updateDriverTab();
    }

    private void handleBottomLimit() {
        stopMotors();
        primaryMotor.setPosition(ElevatorConstants.minPos * ElevatorConstants.rotationsPerInch);
        isHomed = true;
        setpoint = ElevatorConstants.minPos;
        currentState = new TrapezoidProfile.State(ElevatorConstants.minPos, 0);
        goalState = new TrapezoidProfile.State(ElevatorConstants.minPos, 0);
        pidController.reset();
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    public boolean isAtHeight(double targetHeightInches) {
        // Check if the elevator is within a small tolerance of the target height
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - targetHeightInches) < ElevatorConstants.posTolerance;
    }
    

    // private double calculateFeedForward(TrapezoidProfile.State state) {
    //     // kS (static friction), kG (gravity), kV (velocity),
    //     return ElevatorConstants.kS * Math.signum(state.velocity) +
    //            ElevatorConstants.kG +
    //            ElevatorConstants.kV * state.velocity;
    // }

    public void setPositionInches(double inches) {
        if (!isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        setpoint = MathUtil.clamp(
            inches,
            ElevatorConstants.minPos,
            ElevatorConstants.maxPos
        );
        
        // Update goal state for motion profile
        goalState = new TrapezoidProfile.State(setpoint, 0);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Target", setpoint);
        SmartDashboard.putBoolean("Elevator Homed", isHomed);
        SmartDashboard.putString("Elevator State", currentTarget.toString());
        SmartDashboard.putNumber("Elevator Current", primaryMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Velocity", currentState.velocity);
        SmartDashboard.putNumber("Elevator Position", primaryMotor.getPosition().getValueAsDouble());
    }

    public double getHeightInches() {
        return primaryMotor.getPosition().getValueAsDouble() / ElevatorConstants.rotationsPerInch;
    }
// hi! i am so happy im here today :) hg
    public void homeElevator() {
        primaryMotor.set(-0.1); // Slow downward movement until bottom limit is hit
        if (!bottomLimit.get()) {
            handleBottomLimit();
        }
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

    public void moveToSetpoint() {

        if (isHomed) {
            double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
            // double ff = calculateFeedForward(currentState);
            
            double outputPower = MathUtil.clamp(
                pidOutput,
                -ElevatorConstants.max_output,
                ElevatorConstants.max_output
            );
            
            primaryMotor.set(outputPower);
        }
        
    }

    public void setManualPower(double power) {
        // Disable PID control when in manual mode
        pidController.reset();
        currentState = new TrapezoidProfile.State(getHeightInches(), 0);
        goalState = new TrapezoidProfile.State(getHeightInches(), 0);
        
        if (!isHomed && power < 0) {
            power = 0;
        }
        
        if (getHeightInches() >= ElevatorConstants.maxPos && power > 0) {
            power = 0;
        }
        
        if (!bottomLimit.get() && power < 0) {
            power = 0;
        }
        
        primaryMotor.set(MathUtil.clamp(power, -ElevatorConstants.max_output, ElevatorConstants.max_output));
    }

    private void setUpDriverTab() {

        ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Tab");

        elevatorLevels[0] = driverTab.add("L4", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(3, 1)
            .withPosition(0, 0)
            .getEntry();

        elevatorLevels[1] = driverTab.add("L3", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(3, 1)
            .withPosition(0, 1)
            .getEntry();

        elevatorLevels[2] = driverTab.add("L2", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(3, 1)
            .withPosition(0, 2)
            .getEntry();

        elevatorLevels[3] = driverTab.add("L1", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withSize(3, 1)
            .withPosition(0, 3)
            .getEntry();

    }

    // private void updateDriverTab() {

    //     for (int i = 0; i < 4; i++) {

    //         if (elevatorLevels[i].getBoolean(false) != levelBooleans[i] && elevatorLevels[i].getBoolean(false)) {
    //             elevatorLevels[0].setBoolean(false);
    //             levelBooleans[0] = false;
    //             elevatorLevels[1].setBoolean(false);
    //             levelBooleans[1] = false;
    //             elevatorLevels[2].setBoolean(false);
    //             levelBooleans[2] = false;
    //             elevatorLevels[3].setBoolean(false);
    //             levelBooleans[3] = false;

    //             elevatorLevels[i].setBoolean(true);
    //             levelBooleans[i] = true;
    //         } else if (elevatorLevels[i].getBoolean(false) != levelBooleans[i]) {
    //             levelBooleans[i] = false;
    //         }

    //     }

    //     if (levelBooleans[0]) {
    //         currentTarget = ElevatorPosition.POSITION_4;
    //     } else if (levelBooleans[1]) {
    //         currentTarget = ElevatorPosition.POSITION_3;
    //     } else if (levelBooleans[2]) {
    //         currentTarget = ElevatorPosition.POSITION_2;
    //     } else if (levelBooleans[3]) {
    //         currentTarget = ElevatorPosition.POSITION_1;
    //     }

    // }

    public TalonFX getPrimaryMotor() {
        return primaryMotor;
    }
}   