package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climbers.ClimbersSubsystem;

public class ClimbDown extends Command {
    
    private final ClimbersSubsystem m_climber;

    public ClimbDown(ClimbersSubsystem climber) { // the constructor also we need to decide if it is climbers or climber :)

        m_climber = climber;
        addRequirements(m_climber);

    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_climber.setMotor(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stopClimbMotor();
    }
}