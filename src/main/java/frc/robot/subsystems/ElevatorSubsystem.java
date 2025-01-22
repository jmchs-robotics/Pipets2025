package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX primaryMotor;
    private final TalonFX followerMotor;

    public ElevatorSubsystem() {

        primaryMotor = new TalonFX(1);
        followerMotor = new TalonFX(2);

        primaryMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);

        followerMotor.setControl(new Follower(1, true));

    }

    public void setMotors(double speed) {
        primaryMotor.set(speed);
    }

    public void stopMotors() {
        primaryMotor.stopMotor();
    }
    
}
