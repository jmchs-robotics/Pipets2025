package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer.ElevatorLevel;
import frc.robot.subsystems.coral.CoralFlipperSubsystem;
import frc.robot.subsystems.coral.CoralWheelsSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class CommandFactory {
    
    /**
     * Sequence for automatically scoring L4 coral
     * 
     * @param drive - Drive Subsystem
     * @param elevator - Elevator Subsystem
     * @param cFlipper - Coral Flipper Subsystem
     * @param cWheels - Coral Wheels Subsystem
     * @return - The command sequence for L4 scoring.
     */
    public static Command scoreL4(DriveSubsystem drive, ElevatorSubsystem elevator, CoralFlipperSubsystem cFlipper, CoralWheelsSubsystem cWheels) {
        return Commands.parallel(
            Commands.sequence(
                new SetElevator(elevator, ElevatorLevel.LEVEL_4_CORAL)
            ),
            Commands.sequence(
                new SetCoralFlipper(cFlipper, "scoreHigh")
            )
        )
        .andThen(new AlignToPose(drive))
        .andThen(new WaitCommand(0.5))
        .andThen(new CoralExtake(cWheels).withTimeout(0.5))
        .andThen(
            Commands.parallel(
                Commands.sequence(
                    new SetElevator(elevator, ElevatorLevel.HOME),
                    new ZeroElevator(elevator)
                ),
                Commands.sequence(
                    new SetCoralFlipper(cFlipper, "idle"),
                    new ZeroCoralFlipper(cFlipper)
                )
            )
        );
    }

}
