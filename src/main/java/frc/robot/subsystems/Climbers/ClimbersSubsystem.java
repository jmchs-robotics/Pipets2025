package frc.robot.subsystems.Climbers;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimbersSubsystem extends SubsystemBase {

   private final TalonFX climbMotor;

    public ClimbersSubsystem() {

        climbMotor = new TalonFX(ClimberConstants.climbMotorID);

    }

    public void stopClimbMotor() {
        climbMotor.stopMotor();
    }

    public void setMotor(double speed) {
        climbMotor.set(speed);
    }
}