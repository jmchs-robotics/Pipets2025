// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.algae.*;
import frc.robot.subsystems.coral.*;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.AutoSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@Logged
public class RobotContainer {

  // The robot's subsystems
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final AlgaeFlipperSubsystem m_algaeFlipperSubsystem = new AlgaeFlipperSubsystem();
  private final AlgaeWheelsSubsystem m_algaeWheelsSubsystem = new AlgaeWheelsSubsystem();
  private final CoralWheelsSubsystem m_coralWheelsSubsystem = new CoralWheelsSubsystem();
  private final CoralFlipperSubsystem m_coralFlipperSubsystem = new CoralFlipperSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision);
  private final AutoSubsystem m_auto = new AutoSubsystem(m_robotDrive, m_elevatorSubsystem, m_coralFlipperSubsystem, m_coralWheelsSubsystem);
  public final LEDSubsystem m_leds = new LEDSubsystem();

  public static final Field2d field = new Field2d();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  CommandGenericHID buttonBoard = new CommandGenericHID(OIConstants.kButtonBoardPort);

  private final GenericEntry[] elevatorLevels = new GenericEntry[3];
  public static final boolean[] levelBooleans = {true, false, false};

  private final GenericEntry[] reefSides = new GenericEntry[6];
  public static final boolean[] reefSidesBoolean = {true, false, false, false, false, false};

  private final GenericEntry[] reefAlignments = new GenericEntry[3];
  public static final boolean[] reefAlignmentsBoolean = {true, false, false};

  public static ElevatorLevel elevatorLevel = ElevatorLevel.LEVEL_4_CORAL;
  public static ReefSide reefSide = ReefSide.FRONT_LEFT;
  public static ReefAlignment reefAlignment = ReefAlignment.LEFT;
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new DefaultDriveCommand(m_robotDrive, m_driverController));
    m_leds.setDefaultCommand(new DefaultLEDCommand(m_leds));

    setUpDriverTab();
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */


  private void configureButtonBindings() {

    m_driverController.start().onTrue(
      new InstantCommand(() -> {m_robotDrive.zeroHeading();})
    );

    m_driverController.back().onTrue(
      new InstantCommand(() -> {m_elevatorSubsystem.resetSensorPosition(ElevatorConstants.HOME);})
    );

    m_driverController.a().whileTrue(
      new CoralExtake(m_coralWheelsSubsystem)
    );

    m_driverController.b().whileTrue(
      new AlgaeExtake(m_algaeWheelsSubsystem)
    );

    m_driverController.x().onTrue(
      new SequentialCommandGroup(
        new SetCoralFlipper(m_coralFlipperSubsystem, "idle"),
        new ZeroCoralFlipper(m_coralFlipperSubsystem)
      )
    );

    m_driverController.y().onTrue(
      new SequentialCommandGroup(
        new SetAlgaeFlipper(m_algaeFlipperSubsystem, "down"),
        new ZeroAlgaeFlipper(m_algaeFlipperSubsystem)
      )
    );

    m_driverController.rightBumper().onTrue(
      new SequentialCommandGroup(
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
        new ZeroElevator(m_elevatorSubsystem)
      )
    );

    /** commented out climbers as we have no climbers
    *m_driverController.leftBumper().and(() -> climbModeBoolean).whileTrue(
    *  new ClimbDown(m_climbersSubsystem)
    *);
    *
    *m_driverController.leftBumper().and(() -> !climbModeBoolean).whileTrue(
    *  new ClimbUp(m_climbersSubsystem)
    *);
    **/
    
    m_driverController.povUp().whileTrue(
      new AlignToPose(m_robotDrive)
    );

    m_driverController.povDown().and(() -> elevatorLevel == ElevatorLevel.LEVEL_4_CORAL).whileTrue(
      CommandFactory.scoreL4(m_robotDrive, m_elevatorSubsystem, m_coralFlipperSubsystem, m_coralWheelsSubsystem)
    );

    m_driverController.povDown().and(() -> elevatorLevel == ElevatorLevel.LEVEL_3_CORAL).whileTrue(
      new ParallelCommandGroup(
        Commands.sequence(
          // new ZeroElevator(m_elevatorSubsystem),
          new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_3_CORAL)
        ),
        Commands.sequence(
          // new ZeroCoralFlipper(m_coralFlipperSubsystem),
          new SetCoralFlipper(m_coralFlipperSubsystem, "scoreLow")
        )
      )
      .andThen(new AlignToPose(m_robotDrive))
      .andThen(new CoralExtake(m_coralWheelsSubsystem).withTimeout(0.5))
      .andThen(
        new ParallelCommandGroup(
          Commands.sequence(
            new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
            new ZeroElevator(m_elevatorSubsystem)
          ),
          Commands.sequence(
            new SetCoralFlipper(m_coralFlipperSubsystem, "idle"),
            new ZeroCoralFlipper(m_coralFlipperSubsystem)
          )
        )
      )
    );

    m_driverController.povDown().and(() -> elevatorLevel == ElevatorLevel.LEVEL_2_CORAL).whileTrue(
      new ParallelCommandGroup(
        Commands.sequence(
          new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
          new ZeroElevator(m_elevatorSubsystem)
        ),
        Commands.sequence(
          // new ZeroCoralFlipper(m_coralFlipperSubsystem),
          new SetCoralFlipper(m_coralFlipperSubsystem, "scoreLow")
        )
      ).andThen(new AlignToPose(m_robotDrive)
      ).andThen(new CoralExtake(m_coralWheelsSubsystem).withTimeout(0.5))
      .andThen(
        Commands.sequence(
          new SetCoralFlipper(m_coralFlipperSubsystem, "idle"),
          new ZeroCoralFlipper(m_coralFlipperSubsystem)
        )
      )
    );

    m_driverController.povDown().and(() -> elevatorLevel == ElevatorLevel.LEVEL_3_ALGAE).whileTrue(
      new AlgaeIntakeSequence(m_algaeFlipperSubsystem, m_algaeWheelsSubsystem, m_elevatorSubsystem, ElevatorLevel.LEVEL_3_ALGAE)
    );

    m_driverController.povDown().and(() -> elevatorLevel == ElevatorLevel.LEVEL_2_ALGAE).whileTrue(
      new AlgaeIntakeSequence(m_algaeFlipperSubsystem, m_algaeWheelsSubsystem, m_elevatorSubsystem, ElevatorLevel.LEVEL_2_ALGAE)
    );

    m_driverController.povRight().whileTrue(
      new ParallelCommandGroup(
        new SlideRight(m_robotDrive),
        Commands.sequence(
          // new ZeroElevator(m_elevatorSubsystem),
          new SetElevator(m_elevatorSubsystem, ElevatorLevel.CORAL_STATION)
        ),
        Commands.sequence(
          // new ZeroCoralFlipper(m_coralFlipperSubsystem),
          new SetCoralFlipper(m_coralFlipperSubsystem, "coralStation")
        ),
        new CoralIntake(m_coralWheelsSubsystem)
      )
    );

    m_driverController.povLeft().whileTrue(
      new ParallelCommandGroup(
        new SlideLeft(m_robotDrive),
        Commands.sequence(
          // new ZeroElevator(m_elevatorSubsystem),
          new SetElevator(m_elevatorSubsystem, ElevatorLevel.CORAL_STATION)
        ),
        Commands.sequence(
          // new ZeroCoralFlipper(m_coralFlipperSubsystem),
          new SetCoralFlipper(m_coralFlipperSubsystem, "coralStation")
        ),
        new CoralIntake(m_coralWheelsSubsystem)
      )
    );

    m_operatorController.a().whileTrue(
      new LowerCoralFlipperManual(m_coralFlipperSubsystem)
    );

    m_operatorController.b().whileTrue(
      new RaiseCoralFlipperManual(m_coralFlipperSubsystem)
    );

    m_operatorController.x().whileTrue(
      new LowerAlgaeFlipperManual(m_algaeFlipperSubsystem)
    );

    m_operatorController.y().whileTrue(
      new RaiseAlgaeFlipperManual(m_algaeFlipperSubsystem)
    );

    m_operatorController.leftBumper().whileTrue(
      new LowerElevatorManual(m_elevatorSubsystem)
    );

    m_operatorController.rightBumper().whileTrue(
      new RaiseElevatorManual(m_elevatorSubsystem)
    );

    m_operatorController.rightTrigger().onTrue(
      new InstantCommand(() -> m_coralFlipperSubsystem.setPosition(CoralConstants.kIdleAngle), m_coralFlipperSubsystem)
    );

    m_operatorController.leftTrigger().onTrue(
      new InstantCommand(() -> m_algaeFlipperSubsystem.setPosition(AlgaeConstants.kAngleDown), m_algaeFlipperSubsystem)
    );

    m_operatorController.povUp().whileTrue(
      new CoralIntake(m_coralWheelsSubsystem)
    );

    m_operatorController.povDown().whileTrue(
      new CoralExtake(m_coralWheelsSubsystem)
    );

    m_operatorController.povRight().whileTrue(
      new AlgaeIntake(m_algaeWheelsSubsystem)
    );

    m_operatorController.povLeft().whileTrue(
      new AlgaeExtake(m_algaeWheelsSubsystem)
    );

    buttonBoard.button(1).onTrue(
      new InstantCommand(() -> setReefAlignment(0)).ignoringDisable(true)
    );

    buttonBoard.button(2).onTrue(
      new InstantCommand(() -> setReefAlignment(1)).ignoringDisable(true)
    );

    buttonBoard.button(3).onTrue(
      new InstantCommand(() -> setReefAlignment(2)).ignoringDisable(true)
    );

    buttonBoard.button(6).onTrue(
      new InstantCommand(() -> setElevatorLevel(0)).ignoringDisable(true)
    );

    buttonBoard.button(5).onTrue(
      new InstantCommand(() -> setElevatorLevel(1)).ignoringDisable(true)
    );

    buttonBoard.button(4).onTrue(
      new InstantCommand(() -> setElevatorLevel(2)).ignoringDisable(true)
    );

    buttonBoard.button(7).onTrue(
      new InstantCommand(() -> setReefSide(0)).ignoringDisable(true)
    );

    buttonBoard.button(9).onTrue(
      new InstantCommand(() -> setReefSide(1)).ignoringDisable(true)
    );

    buttonBoard.button(11).onTrue(
      new InstantCommand(() -> setReefSide(2)).ignoringDisable(true)
    );

    buttonBoard.button(8).onTrue(
      new InstantCommand(() -> setReefSide(3)).ignoringDisable(true)
    );

    buttonBoard.button(10).onTrue(
      new InstantCommand(() -> setReefSide(4)).ignoringDisable(true)
    );

    buttonBoard.button(12).onTrue(
      new InstantCommand(() -> setReefSide(5)).ignoringDisable(true)
    );
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_auto.getAutoCommand();
  }

  private void setUpDriverTab() {

    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver Tab");

    elevatorLevels[0] = driverTab.add("L4", true)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(3, 1)
      .withPosition(0, 0)
      .getEntry();

    elevatorLevels[1] = driverTab.add("L3", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(3, 1)
      .withPosition(0, 1)
      .getEntry();

    elevatorLevels[2] = driverTab.add("L2", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(3, 1)
      .withPosition(0, 2)
      .getEntry();

    reefSides[0] = driverTab.add("FL", true)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(1, 1)
      .withPosition(3, 0)
      .getEntry();

    reefSides[1] = driverTab.add("FM", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(1, 1)
      .withPosition(4, 0)
      .getEntry();

    reefSides[2] = driverTab.add("FR", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(1, 1)
      .withPosition(5, 0)
      .getEntry();

    reefSides[3] = driverTab.add("BL", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(1, 1)
      .withPosition(3, 1)
      .getEntry();

    reefSides[4] = driverTab.add("BM", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(1, 1)
      .withPosition(4, 1)
      .getEntry();

    reefSides[5] = driverTab.add("BR", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(1, 1)
      .withPosition(5, 1)
      .getEntry();

    reefAlignments[0] = driverTab.add("Left", true)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(1, 1)
      .withPosition(3, 2)
      .getEntry();

    reefAlignments[1] = driverTab.add("Center", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(1, 1)
      .withPosition(4, 2)
      .getEntry();

    reefAlignments[2] = driverTab.add("Right", false)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withSize(1, 1)
      .withPosition(5, 2)
      .getEntry();
      
  }

  public void updateDriverTab() {

    for (int i = 0; i < 3; i++) {
      if (levelBooleans[i]) {
        elevatorLevels[i].setBoolean(true);
      } else {
        elevatorLevels[i].setBoolean(false);
      }
    }

    for (int i = 0; i < 6; i++) {
      if (reefSidesBoolean[i]) {
        reefSides[i].setBoolean(true);
      } else {
        reefSides[i].setBoolean(false);
      }
    }

    for (int i = 0; i < 3; i++) {
      if (reefAlignmentsBoolean[i]) {
        reefAlignments[i].setBoolean(true);
      } else {
        reefAlignments[i].setBoolean(false);
      }
    }

    SmartDashboard.putString("Elevator Level", elevatorLevel.toString());
    SmartDashboard.putString("Reef Side", reefSide.toString());

  }

  public void decideElevatorPosition() {

    elevatorLevel = ElevatorLevel.LEVEL_4_CORAL;

    if (levelBooleans[0]) {
      elevatorLevel = ElevatorLevel.LEVEL_4_CORAL;
    } else if (levelBooleans[1] && (reefAlignmentsBoolean[0] || reefAlignmentsBoolean[2])) {
      elevatorLevel = ElevatorLevel.LEVEL_3_CORAL;
    } else if (levelBooleans[1] && reefAlignmentsBoolean[1]) {
      elevatorLevel = ElevatorLevel.LEVEL_3_ALGAE;
    } else if (levelBooleans[2] && (reefAlignmentsBoolean[0] || reefAlignmentsBoolean[2])) {
      elevatorLevel = ElevatorLevel.LEVEL_2_CORAL;
    } else if (levelBooleans[2] && reefAlignmentsBoolean[1]) {
      elevatorLevel = ElevatorLevel.LEVEL_2_ALGAE;
    }

    if (reefSidesBoolean[0]) {
      reefSide = ReefSide.FRONT_LEFT;
    } else if (reefSidesBoolean[1]) {
      reefSide = ReefSide.FRONT_MIDDLE;
    } else if (reefSidesBoolean[2]) {
      reefSide = ReefSide.FRONT_RIGHT;
    } else if (reefSidesBoolean[3]) {
      reefSide = ReefSide.BACK_LEFT;
    } else if (reefSidesBoolean[4]) {
      reefSide = ReefSide.BACK_MIDDLE;
    } else if (reefSidesBoolean[5]) {
      reefSide = ReefSide.BACK_RIGHT;
    }

    for (int i = 0; i < 3; i++) {
      if (reefAlignmentsBoolean[i]) {
        reefAlignment = ReefAlignment.values()[i];
      }
    }

  }

  private void setElevatorLevel(int num) {
    for (int i = 0; i < levelBooleans.length; i++) {
      levelBooleans[i] = false;
    }

    levelBooleans[num] = true;
  }

  private void setReefAlignment(int num) {
    for (int i = 0; i < reefAlignmentsBoolean.length; i++) {
      reefAlignmentsBoolean[i] = false;
    }

    reefAlignmentsBoolean[num] = true;
  }

  private void setReefSide(int num) {
    for (int i = 0; i < reefSidesBoolean.length; i++) {
      reefSidesBoolean[i] = false;
    }

    reefSidesBoolean[num] = true;
  }

  public enum ElevatorLevel {
    LEVEL_4_CORAL,
    LEVEL_3_CORAL,
    LEVEL_2_CORAL,
    LEVEL_3_ALGAE,
    LEVEL_2_ALGAE,
    CORAL_STATION,
    HOME
  }

  public enum ReefSide {
    FRONT_LEFT,
    FRONT_MIDDLE,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_MIDDLE,
    BACK_RIGHT
  }

  public enum ReefAlignment {
    LEFT,
    CENTER,
    RIGHT
  }

}
