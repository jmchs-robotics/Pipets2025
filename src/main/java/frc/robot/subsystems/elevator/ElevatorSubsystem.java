package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    
    private final TalonFX primaryMotor;
    private final TalonFX followerMotor;
    private final TalonFXConfiguration config;

    public ElevatorSubsystem() {

        primaryMotor = new TalonFX(ElevatorConstants.primaryElevatorID);
        followerMotor = new TalonFX(ElevatorConstants.followerElevatorID);
        config = new TalonFXConfiguration();

        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kI = ElevatorConstants.kI;
        config.Slot0.kD = ElevatorConstants.kD;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40; // 40 amp breaker on PDH
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(ElevatorConstants.maxPos * ElevatorConstants.rotationsPerInch).in(Units.Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(ElevatorConstants.minPos * ElevatorConstants.rotationsPerInch).in(Units.Rotations);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        config.Feedback.SensorToMechanismRatio = 9; // 9:1 gear ratio -> TODO: Maybe try to replace it with RotationsToInches ratio?

        primaryMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

    }

    public void setPosition(Angle height) {
        primaryMotor.setControl(new PositionVoltage(height.in(Units.Rotations)));
        followerMotor.setControl(new Follower(primaryMotor.getDeviceID(), true));
    }

    public void setNeutral() {
        primaryMotor.setControl(new NeutralOut());
        followerMotor.setControl(new NeutralOut());
    }

    public void resetSensorPosition(Angle setpoint) {
        primaryMotor.setPosition(setpoint.in(Units.Rotations));

        // TODO: Generally elevators get pretty mad if you try to set it negative and then call positive setpoints...
        // Maybe create a new config with config.MotorOutput.Inverted to CW+?
        // Or just delete this line since you're never going to call on the follower motor position
        followerMotor.setPosition(-setpoint.in(Units.Rotations)); 
    }
}
