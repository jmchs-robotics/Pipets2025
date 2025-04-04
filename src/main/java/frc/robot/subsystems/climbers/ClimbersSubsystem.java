package frc.robot.subsystems.climbers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimbersSubsystem extends SubsystemBase {

   private final TalonFX climbMotor;

    public ClimbersSubsystem() {

        climbMotor = new TalonFX(ClimberConstants.climbMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbMotor.getConfigurator().apply(config);

    }

    public void stopClimbMotor() {
        climbMotor.stopMotor();
    }

    public void setMotor(double speed) {
        climbMotor.set(speed);
    }
}