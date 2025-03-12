package frc.robot.subsystems.coral;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        config.Feedback.SensorToMechanismRatio = 36; // 36:1 gear ratio

        flipMotor.getConfigurator().apply(config);

        flipMotor.setPosition(0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Flipper Motor Rotations", flipMotor.getPosition().getValueAsDouble());
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

    public double getCurrentPosition() {
        return flipMotor.getPosition().getValueAsDouble();
    }

    public double getCurrentVelocity() {
        return flipMotor.getVelocity().getValueAsDouble();
    }
}
