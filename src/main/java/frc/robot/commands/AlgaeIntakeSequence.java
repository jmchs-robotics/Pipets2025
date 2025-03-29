package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer.ElevatorLevel;
import frc.robot.subsystems.algae.AlgaeFlipperSubsystem;
import frc.robot.subsystems.algae.AlgaeWheelsSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class AlgaeIntakeSequence extends Command {

    private final AlgaeFlipperSubsystem m_algaeFlipper;
    private final AlgaeWheelsSubsystem m_wheels;
    private final ElevatorSubsystem m_elevator;
    private final ElevatorLevel m_level;
    private Timer timer = new Timer();

    public AlgaeIntakeSequence(AlgaeFlipperSubsystem algaeFlipper, AlgaeWheelsSubsystem wheels, ElevatorSubsystem elevator, ElevatorLevel level) {

        m_algaeFlipper = algaeFlipper;
        m_wheels = wheels;
        m_elevator = elevator;
        m_level = level;

        addRequirements(algaeFlipper, wheels, elevator);

    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {

        if (m_level == ElevatorLevel.LEVEL_3_ALGAE) {
            m_elevator.setPosition(ElevatorConstants.L3_ALGAE);
        } else if (m_level == ElevatorLevel.LEVEL_2_ALGAE) {
            m_elevator.setPosition(ElevatorConstants.L2_ALGAE);
        }

        if (timer.get() > 0.25) {
            m_algaeFlipper.setPosition(AlgaeConstants.kAngleUp);
        }

        m_wheels.setWheelMotors(-0.5);

    }

    @Override
    public void end(boolean interrupted) {

        timer.reset();

        while (timer.get() < 0.5) {
            m_elevator.setElevatorManual(0.1);
            m_wheels.setWheelMotors(-0.5);
        }

        m_wheels.stopWheelMotors();

        if (m_level == ElevatorLevel.LEVEL_3_ALGAE) {
            m_elevator.setPosition(ElevatorConstants.L3_ALGAE);
        } else if (m_level == ElevatorLevel.LEVEL_2_ALGAE) {
            m_elevator.setPosition(ElevatorConstants.L2_ALGAE);
        }

    }
    
}
