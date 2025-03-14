package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer.ElevatorLevel;
import frc.robot.commands.CoralExtake;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.SetCoralFlipper;
import frc.robot.commands.SetElevator;
import frc.robot.subsystems.algae.AlgaeFlipperSubsystem;
import frc.robot.subsystems.algae.AlgaeWheelsSubsystem;
import frc.robot.subsystems.coral.CoralFlipperSubsystem;
import frc.robot.subsystems.coral.CoralWheelsSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class AutoSubsystem extends SubsystemBase {

    private ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();
    private Command autoCommand = Commands.runOnce(() -> {});
    private String feedback = "Enter Auto Path Sequence";

    GenericEntry autoEntry;

    private DriveSubsystem m_driveSubsystem;
    private ElevatorSubsystem m_elevatorSubsystem;
    private CoralFlipperSubsystem m_coralFlipper;
    private CoralWheelsSubsystem m_coralWheels;

    private char[] REEF_SPOTS = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L'};
    private char[] CORAL_STATION_SPOTS = {'V', 'W'};

    public AutoSubsystem(
        DriveSubsystem drive,
        ElevatorSubsystem elevator,
        CoralFlipperSubsystem coralFlipper,
        CoralWheelsSubsystem coralWheels) {

            NetworkTable table = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Auto Tab");
            autoEntry = table.getTopic("Auto Path Sequence").getGenericEntry();
            
            m_driveSubsystem = drive;
            m_elevatorSubsystem = elevator;
            m_coralFlipper = coralFlipper;
            m_coralWheels = coralWheels;

            setUpAutoTab();
    }

    @Override
    public void periodic() {
        // validateAndCreatePaths();
    }

    public Command getAutoCommand() {
        validateAndCreatePaths();
        return autoCommand;
    }

    public void setUpAutoTab() {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Tab");

        autoTab.add("Auto Path Sequence", "").withSize(3, 1).withPosition(0, 0);
        autoTab.add(RobotContainer.field).withSize(6, 4).withPosition(3, 0);
        autoTab.addString("Feedback", () -> feedback).withSize(3, 1).withPosition(0, 1);

        NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Auto Tab");

        ntTable.addListener(
            "Auto Path Sequence",
            EnumSet.of(Kind.kValueAll),
            (table, key, event) -> {
                validateAndCreatePaths();
            }
        );
    }

    public void setFeedback(String val) {
        feedback = val;
    }

    public String getFeedback() {
        return feedback;
    }

    // public void drawPaths() {
    //     clearField();
    //     for (int i = 0; i < trajectories.size(); i++) {
    //         PathPlannerTrajectory pathTraj = trajectories.get(i);
    //         List<State> states = convertStatesToStates(pathTraj.getStates());
    //         Trajectory displayTrajectory = new Trajectory(states);

    //         RobotContainer.field.getObject("traj" + i).setTrajectory(displayTrajectory);
    //     }
    // }

    public void clearField() {
        for (int i = 0; i < 100; i++) {
            FieldObject2d obj = RobotContainer.field.getObject("traj" + i);
            obj.setTrajectory(new Trajectory());
        }
    }

    public void clearAll() {
        trajectories.clear();
        clearField();
    }

    // public List<State> convertStatesToStates(List<PathPlannerTrajectoryState> ppStates) {
    //     ArrayList<State> wpiStates = new ArrayList<State>();
        
    //     for (int i = 0; i < ppStates.size(); i++) {
    //         PathPlannerTrajectoryState currentState = ppStates.get(i);
    //         if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
    //             wpiStates.add(new State(
    //                 currentState.timeSeconds,
    //                 currentState.linearVelocity,
    //                 currentState.linearVelocity / currentState.timeSeconds,
    //                 currentState.flip().pose,
    //                 currentState.curvatureRadPerMeter
    //             ));
    //         } else {
    //             wpiStates.add(new State(
    //                 currentState.timeSeconds,
    //                 currentState.linearVelocity,
    //                 currentState.linearVelocity / currentState.timeSeconds,
    //                 currentState.pose,
    //                 currentState.curvatureRadPerMeter
    //             ));
    //         }
    //     }

    //     return wpiStates;
    // }

    public void validateAndCreatePaths() {
        String autoString = autoEntry.getString("");

        buildPathSequenceOdometry(autoString);
        // drawPaths();
    }

    public void buildPathSequenceOdometry(String autoString) {

        SequentialCommandGroup finalPath = new SequentialCommandGroup();

        trajectories.clear();

        if (autoString.length() <= 1) {
            setFeedback("Default Path (Do nothing and cry)");
            return;
        }

        // TODO: Put in the rest of the starting points
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            if (autoString.charAt(0) == '1') {
                m_driveSubsystem.resetOdometry(new Pose2d(10.38, 0.500, Rotation2d.fromDegrees(0)));
            } else if (autoString.charAt(0) == '2') {
                m_driveSubsystem.resetOdometry(new Pose2d(10.38, 1.890, Rotation2d.fromDegrees(0)));
            } else if (autoString.charAt(0) == '3') {
                m_driveSubsystem.resetOdometry(new Pose2d(10.38, 3.300, Rotation2d.fromDegrees(0)));
            } else if (autoString.charAt(0) == '4') {
                m_driveSubsystem.resetOdometry(new Pose2d(10.38, 4.750, Rotation2d.fromDegrees(0)));
            } else if (autoString.charAt(0) == '5') {
                m_driveSubsystem.resetOdometry(new Pose2d(10.38, 6.160, Rotation2d.fromDegrees(0)));
            } else if (autoString.charAt(0) == '6') {
                m_driveSubsystem.resetOdometry(new Pose2d(10.38, 7.550, Rotation2d.fromDegrees(0)));
            }
        } else {
            if (autoString.charAt(0) == '1') {
                m_driveSubsystem.resetOdometry(new Pose2d(7.170, 7.550, Rotation2d.fromDegrees(180)));
            } else if (autoString.charAt(0) == '2') {
                m_driveSubsystem.resetOdometry(new Pose2d(7.170, 6.160, Rotation2d.fromDegrees(180)));
            } else if (autoString.charAt(0) == '3') {
                m_driveSubsystem.resetOdometry(new Pose2d(7.170, 4.750, Rotation2d.fromDegrees(180)));
            } else if (autoString.charAt(0) == '4') {
                m_driveSubsystem.resetOdometry(new Pose2d(7.170, 3.300, Rotation2d.fromDegrees(180)));
            } else if (autoString.charAt(0) == '5') {
                m_driveSubsystem.resetOdometry(new Pose2d(7.170, 1.890, Rotation2d.fromDegrees(180)));
            } else if (autoString.charAt(0) == '6') {
                m_driveSubsystem.resetOdometry(new Pose2d(7.170, 0.500, Rotation2d.fromDegrees(180)));
            }
        }

        ParallelRaceGroup segment = new ParallelRaceGroup();
        for (int i = 0; i < autoString.length() - 1; i++) {
            segment = new ParallelRaceGroup();
            char currentPoint = autoString.charAt(i);
            char nextPoint = autoString.charAt(i + 1);

            try {
                if (nextPoint != currentPoint) {
                    PathPlannerPath path = PathPlannerPath.fromPathFile("PP" + currentPoint + "-" + nextPoint);  
                    trajectories.add(path.getIdealTrajectory(m_driveSubsystem.getRobotConfig()).get());
                    Command cmd = Commands.sequence(AutoBuilder.followPath(path), new WaitCommand(0.25));
                    segment = new ParallelRaceGroup(cmd);
                }
            } catch (Exception e) {
                setFeedback("Couldn't Find Path File");
                autoCommand = Commands.runOnce(() -> {});
                return;
            }          

            finalPath.addCommands(segment);

            // If we're at the reef, then do this whole scoring sequence
            if (indexOfAutoChar(REEF_SPOTS, nextPoint) != -1) {
                finalPath.addCommands(
                    Commands.sequence(
                        // Raise elevator to L4 and Lower Coral
                        Commands.parallel(
                            new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_4_CORAL),
                            new SetCoralFlipper(m_coralFlipper, "scoreHigh")
                        ),
                        // Give time for them to raise up b/c they technically finish instantly
                        new WaitCommand(1.25),
                        // Score the coral
                        new CoralExtake(m_coralWheels).withTimeout(0.5),
                        // Bring the elevator back down and store coral
                        Commands.parallel(
                            new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
                            new SetCoralFlipper(m_coralFlipper, "idle")
                        ),
                        new WaitCommand(0.5)
                    )
                );
            }

            // If we're at the coral station,
            if (indexOfAutoChar(CORAL_STATION_SPOTS, nextPoint) != -1) {
                finalPath.addCommands(
                    Commands.sequence(
                        // Raise elevator to the coral station level
                        Commands.parallel(
                            new SetElevator(m_elevatorSubsystem, ElevatorLevel.CORAL_STATION),
                            new SetCoralFlipper(m_coralFlipper, "coralStation")
                        ),
                        // Give it time to raise
                        new WaitCommand(0.5),
                        // Give a lot of time to intake so the human play has time to throw the piece
                        new CoralIntake(m_coralWheels).withTimeout(1.5),
                        // Put the elevator back so we can drive
                        Commands.parallel(
                            new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
                            new SetCoralFlipper(m_coralFlipper, "idle")
                        ),
                        new WaitCommand(0.5)
                    )
                );
            }
            
        }

        autoCommand = finalPath;
        setFeedback("Created Path Sequence");
    }

    public int indexOfAutoChar(char[] arr, char val) {
        for (int i = 0; i < arr.length; i++) {
            if (arr[i] == val) {
                return i;
            }
        }
        return -1;
    }
    
}