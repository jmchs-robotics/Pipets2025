package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeFlipperSubsystem extends SubsystemBase {
    
    private final TalonFX flipMotor;
    private final TalonFXConfiguration config;

    public AlgaeFlipperSubsystem() {

        flipMotor = new TalonFX(AlgaeConstants.flipMotorID);
        config = new TalonFXConfiguration();

        config.Slot0.kP = AlgaeConstants.kP;
        config.Slot0.kI = AlgaeConstants.kI;
        config.Slot0.kD = AlgaeConstants.kD;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40; // 40 amp breaker on PDH
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(0).in(Units.Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(-0.25).in(Units.Rotations);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = 45; // 45:1 gear ratio

        flipMotor.getConfigurator().apply(config);

        flipMotor.setPosition(0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Flip Motor Position", flipMotor.getPosition().getValueAsDouble());
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
