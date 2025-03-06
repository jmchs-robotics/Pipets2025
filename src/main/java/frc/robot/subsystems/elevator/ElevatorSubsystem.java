package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
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
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(ElevatorConstants.maxPos).in(Units.Inches);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(ElevatorConstants.minPos).in(Units.Inches);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = 1/9; // 9:1 gear ratio 

        primaryMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

    }

    public void setPosition(Distance height) {
        primaryMotor.setControl(new PositionVoltage(height.in(Units.Inches)));
        followerMotor.setControl(new Follower(primaryMotor.getDeviceID(), true));
    }

    public void setNeutral() {
        primaryMotor.setControl(new NeutralOut());
        followerMotor.setControl(new NeutralOut());
    }

    public void resetSensorPosition(Distance setpoint) {
        primaryMotor.setPosition(setpoint.in(Units.Inches));
        followerMotor.setPosition(-setpoint.in(Units.Inches));
    }
}
