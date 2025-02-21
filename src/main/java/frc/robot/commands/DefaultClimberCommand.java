package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climbers.ClimbersSubsystem;

public class DefaultClimberCommand extends Command {
    
    private final ClimbersSubsystem m_climber;

    public DefaultClimberCommand(ClimbersSubsystem climber) { // the constructor also we need to decide if it is climbers or climber :)

        m_climber = climber;
        addRequirements(m_climber);

    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_climber.climbDown();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}