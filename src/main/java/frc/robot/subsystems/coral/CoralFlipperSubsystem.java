package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralFlipperSubsystem extends SubsystemBase {
    
    private final TalonFX flipMotor;
    private final TalonFXConfiguration config;

    public CoralFlipperSubsystem() {

        flipMotor = new TalonFX(CoralConstants.flipMotorID);
        config = new TalonFXConfiguration();

        config.Slot0.kP = CoralConstants.kP;
        config.Slot0.kI = CoralConstants.kI;
        config.Slot0.kD = CoralConstants.kD;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40; // 40 amp breaker on PDH
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Degrees.of(135).in(Units.Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Degrees.of(0).in(Units.Rotations);

        config.Feedback.SensorToMechanismRatio = 12; // 12:1 gear ratio

        flipMotor.getConfigurator().apply(config);

    }

    public void setPosition(Angle angle) {
        flipMotor.setControl(new PositionVoltage(angle.in(Units.Rotations)));
    }

    public void setNeutral() {
        flipMotor.setControl(new NeutralOut());
    }

    public void resetSensorPosition(Angle setpoint) {
        flipMotor.setPosition(setpoint.in(Units.Rotations));
    }
}
