// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.algae.*;
import frc.robot.subsystems.coral.*;
import frc.robot.subsystems.climbers.*;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.AutoSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
  private final ClimbersSubsystem m_climbersSubsystem = new ClimbersSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision);
  private final AutoSubsystem m_auto = new AutoSubsystem(m_robotDrive, m_elevatorSubsystem, m_coralFlipperSubsystem, m_coralWheelsSubsystem);
  public final LEDSubsystem m_leds = new LEDSubsystem();

  public static final Field2d field = new Field2d();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  JoystickButton driveStart = new JoystickButton(m_driverController, Button.kStart.value);
  JoystickButton driveBack = new JoystickButton(m_driverController, Button.kBack.value);
  JoystickButton driveA = new JoystickButton(m_driverController, Button.kA.value);
  JoystickButton driveB = new JoystickButton(m_driverController, Button.kB.value);
  JoystickButton driveX = new JoystickButton(m_driverController, Button.kX.value);
  JoystickButton driveY = new JoystickButton(m_driverController, Button.kY.value);
  JoystickButton driveRB = new JoystickButton(m_driverController, Button.kRightBumper.value);
  JoystickButton driveLB = new JoystickButton(m_driverController, Button.kLeftBumper.value);
  POVButton driveUpDPad = new POVButton(m_driverController, 0);
  POVButton driveDownDPad = new POVButton(m_driverController, 180);
  POVButton driveRightDPad = new POVButton(m_driverController, 90);
  POVButton driveLeftDPad = new POVButton(m_driverController, 270);

  JoystickButton operatorA = new JoystickButton(m_operatorController, Button.kA.value);
  JoystickButton operatorB = new JoystickButton(m_operatorController, Button.kB.value);
  JoystickButton operatorX = new JoystickButton(m_operatorController, Button.kX.value);
  JoystickButton operatorY = new JoystickButton(m_operatorController, Button.kY.value);
  JoystickButton operatorRB = new JoystickButton(m_operatorController, Button.kRightBumper.value);
  JoystickButton operatorLB = new JoystickButton(m_operatorController, Button.kLeftBumper.value);
  POVButton operatorUpDPad = new POVButton(m_operatorController, 0);
  POVButton operatorDownDPad = new POVButton(m_operatorController, 180);
  POVButton operatorRightDPad = new POVButton(m_operatorController, 90);
  POVButton operatorLeftDPad = new POVButton(m_operatorController, 270); 

  private final GenericEntry[] elevatorLevels = new GenericEntry[4];
  public static final boolean[] levelBooleans = new boolean[4];

  private final GenericEntry[] reefSides = new GenericEntry[6];
  public static final boolean[] reefSidesBoolean = new boolean[6];

  private final GenericEntry[] reefAlignments = new GenericEntry[3];
  public static final boolean[] reefAlignmentsBoolean = new boolean[3];

  private GenericEntry climbMode;
  private static boolean climbModeBoolean = false;

  public static ElevatorLevel elevatorLevel;
  public static ReefSide reefSide;
  public static ReefAlignment reefAlignment;
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new DefaultDriveCommand(m_robotDrive, m_driverController));
    m_leds.setDefaultCommand(new DefaultLEDCommand(m_leds));
    m_coralWheelsSubsystem.setDefaultCommand(new DefaultCoralWheelsCommand(m_coralWheelsSubsystem));

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

    driveStart.onTrue(
      new InstantCommand(() -> {m_robotDrive.zeroHeading();})
    );

    driveBack.onTrue(
      new InstantCommand(() -> {m_elevatorSubsystem.resetSensorPosition(ElevatorConstants.HOME);})
    );

    driveA.whileTrue(
      new CoralExtake(m_coralWheelsSubsystem)
    );

    driveB.whileTrue(
      new AlgaeExtake(m_algaeWheelsSubsystem)
    );

    driveX.onTrue(
      new SequentialCommandGroup(
        new SetCoralFlipper(m_coralFlipperSubsystem, "idle"),
        new ZeroCoralFlipper(m_coralFlipperSubsystem)
      )
    );

    driveY.onTrue(
      new SequentialCommandGroup(
        new SetAlgaeFlipper(m_algaeFlipperSubsystem, "down"),
        new ZeroAlgaeFlipper(m_algaeFlipperSubsystem)
      )
    );

    driveRB.onTrue(
      new SequentialCommandGroup(
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
        new ZeroElevator(m_elevatorSubsystem)
      )
    );

    driveLB.and(() -> !climbModeBoolean).whileTrue( //the up and down commands were swapped so I swapped them correctly
      new ClimbDown(m_climbersSubsystem)
    );

    driveLB.and(() -> climbModeBoolean).whileTrue(
      new ClimbUp(m_climbersSubsystem)
    );
    
    driveUpDPad.whileTrue(
      new AlignToReef(m_robotDrive, m_vision.getCamera(0))
    );

    driveDownDPad.and(() -> elevatorLevel == ElevatorLevel.LEVEL_4_CORAL).whileTrue(
      new ParallelCommandGroup(
        // m_robotDrive.pathFindToReefCoral(),
        new ParallelCommandGroup(
          Commands.sequence(
            // new ZeroElevator(m_elevatorSubsystem),
            new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_4_CORAL)
          ),
          Commands.sequence(
            // new ZeroCoralFlipper(m_coralFlipperSubsystem),
            new SetCoralFlipper(m_coralFlipperSubsystem, "scoreHigh")
          )
        )
      )
    );

    driveDownDPad.and(() -> elevatorLevel == ElevatorLevel.LEVEL_3_CORAL).whileTrue(
      new ParallelCommandGroup(
        // m_robotDrive.pathFindToReefCoral(),
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
      )
    );

    driveDownDPad.and(() -> elevatorLevel == ElevatorLevel.LEVEL_2_CORAL).whileTrue(
      new ParallelCommandGroup(
        // m_robotDrive.pathFindToReefCoral(),
        new ParallelCommandGroup(
          Commands.sequence(
            new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
            new ZeroElevator(m_elevatorSubsystem)
          ),
          Commands.sequence(
            // new ZeroCoralFlipper(m_coralFlipperSubsystem),
            new SetCoralFlipper(m_coralFlipperSubsystem, "scoreLow")
          )
        )
      )
    );

    driveDownDPad.and(() -> elevatorLevel == ElevatorLevel.LEVEL_3_ALGAE).whileTrue(
      new ParallelCommandGroup(
        // m_robotDrive.pathFindToReefAlgae(),
        new SequentialCommandGroup(
          Commands.sequence(
            // new ZeroElevator(m_elevatorSubsystem),
            new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_3_ALGAE)
          ),
          new WaitCommand(0.25),
          Commands.sequence(
            // new ZeroAlgaeFlipper(m_algaeFlipperSubsystem),
            new SetAlgaeFlipper(m_algaeFlipperSubsystem, "up")
          ),
          new AlgaeIntake(m_algaeWheelsSubsystem)
        )
      )
    );

    driveDownDPad.and(() -> elevatorLevel == ElevatorLevel.LEVEL_2_ALGAE).whileTrue(
      new ParallelCommandGroup(
        // m_robotDrive.pathFindToReefAlgae(),
        new SequentialCommandGroup(
          Commands.sequence(
            // new ZeroElevator(m_elevatorSubsystem),
            new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_2_ALGAE)
          ),
          new WaitCommand(0.25),
          Commands.sequence(
            // new ZeroAlgaeFlipper(m_algaeFlipperSubsystem),
            new SetAlgaeFlipper(m_algaeFlipperSubsystem, "up")
          ),
          new AlgaeIntake(m_algaeWheelsSubsystem)
        )
      )
    );

    driveRightDPad.whileTrue(
      new ParallelCommandGroup(
        new SlideRight(m_robotDrive),
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.CORAL_STATION),
        new SetCoralFlipper(m_coralFlipperSubsystem, "coralStation"),
        new CoralIntake(m_coralWheelsSubsystem)
      )
    );

    driveLeftDPad.whileTrue(
      new ParallelCommandGroup(
        new SlideLeft(m_robotDrive),
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.CORAL_STATION),
        new SetCoralFlipper(m_coralFlipperSubsystem, "coralStation"),
        new CoralIntake(m_coralWheelsSubsystem)
      )
    );

    operatorA.whileTrue(
      new LowerCoralFlipperManual(m_coralFlipperSubsystem)
    );

    operatorB.whileTrue(
      new RaiseCoralFlipperManual(m_coralFlipperSubsystem)
    );

    operatorX.whileTrue(
      new LowerAlgaeFlipperManual(m_algaeFlipperSubsystem)
    );

    operatorY.whileTrue(
      new RaiseAlgaeFlipperManual(m_algaeFlipperSubsystem)
    );

    operatorLB.whileTrue(
      new LowerElevatorManual(m_elevatorSubsystem)
    );

    operatorRB.whileTrue(
      new RaiseElevatorManual(m_elevatorSubsystem)
    );

    operatorUpDPad.whileTrue(
      new CoralIntake(m_coralWheelsSubsystem)
    );

    operatorDownDPad.whileTrue(
      new CoralExtake(m_coralWheelsSubsystem)
    );

    operatorRightDPad.whileTrue(
      new AlgaeIntake(m_algaeWheelsSubsystem)
    );

    operatorLeftDPad.whileTrue(
      new AlgaeExtake(m_algaeWheelsSubsystem)
    );

    // driveA.whileTrue(
    //   new CoralIntake(m_coralWheelsSubsystem)
    // );

    // driveB.whileTrue(
    //   new CoralExtake(m_coralWheelsSubsystem)
    // );

    // driveX.onTrue(
    //   new SetCoralFlipper(m_coralFlipperSubsystem, "coralStation")
    // );

    // driveY.onTrue(
    //   new SetCoralFlipper(m_coralFlipperSubsystem, "idle")
    // );

    // driveRB.onTrue(
    //   new SetElevator(m_elevatorSubsystem, ElevatorLevel.CORAL_STATION)
    // );

    // driveLB.onTrue(
    //   new SequentialCommandGroup(
    //     new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
    //     new ZeroElevator(m_elevatorSubsystem)
    //   )
    // );
    
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
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(3, 1)
          .withPosition(0, 0)
          .getEntry();

        elevatorLevels[1] = driverTab.add("L3", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(3, 1)
          .withPosition(0, 1)
          .getEntry();

        elevatorLevels[2] = driverTab.add("L2", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(3, 1)
          .withPosition(0, 2)
          .getEntry();

        elevatorLevels[3] = driverTab.add("L1", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(3, 1)
          .withPosition(0, 3)
          .getEntry();

        reefSides[0] = driverTab.add("FL", true)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(3, 0)
          .getEntry();

        reefSides[1] = driverTab.add("FM", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(4, 0)
          .getEntry();

        reefSides[2] = driverTab.add("FR", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(5, 0)
          .getEntry();

        reefSides[3] = driverTab.add("BL", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(3, 1)
          .getEntry();

        reefSides[4] = driverTab.add("BM", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(4, 1)
          .getEntry();

        reefSides[5] = driverTab.add("BR", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(5, 1)
          .getEntry();

        reefAlignments[0] = driverTab.add("Left", true)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(3, 2)
          .getEntry();

        reefAlignments[1] = driverTab.add("Center", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(4, 2)
          .getEntry();

        reefAlignments[2] = driverTab.add("Right", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(5, 2)
          .getEntry();

        climbMode = driverTab.add("Climb Mode", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(1, 1)
          .withPosition(6, 0)
          .getEntry();

    }

    public void updateDriverTab() {

      for (int i = 0; i < 4; i++) {
        if (elevatorLevels[i].getBoolean(false) != levelBooleans[i] & elevatorLevels[i].getBoolean(false)) {
          elevatorLevels[0].setBoolean(false);
          levelBooleans[0] = false;
          elevatorLevels[1].setBoolean(false);
          levelBooleans[1] = false;
          elevatorLevels[2].setBoolean(false);
          levelBooleans[2] = false;
          elevatorLevels[3].setBoolean(false);
          levelBooleans[3] = false;
          elevatorLevels[i].setBoolean(true);
          levelBooleans[i] = true;
        } else if (elevatorLevels[i].getBoolean(false) != levelBooleans[i]) {
          levelBooleans[i] = false;
        }
      }

      for (int i = 0; i < 6; i++) {
        if (reefSides[i].getBoolean(false) != reefSidesBoolean[i] & reefSides[i].getBoolean(false)) {
          reefSides[0].setBoolean(false);
          reefSidesBoolean[0] = false;
          reefSides[1].setBoolean(false);
          reefSidesBoolean[1] = false;
          reefSides[2].setBoolean(false);
          reefSidesBoolean[2] = false;
          reefSides[3].setBoolean(false);
          reefSidesBoolean[3] = false;
          reefSides[4].setBoolean(false);
          reefSidesBoolean[4] = false;
          reefSides[5].setBoolean(false);
          reefSidesBoolean[5] = false;
          reefSides[i].setBoolean(true);
          reefSidesBoolean[i] = true;
        } else if (reefSides[i].getBoolean(false) != reefSidesBoolean[i]) {
          reefSidesBoolean[i] = false;
        }
      }

      for (int i = 0; i < 3; i++) {
        if (reefAlignments[i].getBoolean(false) != reefAlignmentsBoolean[i] & reefAlignments[i].getBoolean(false)) {
          reefAlignments[0].setBoolean(false);
          reefAlignmentsBoolean[0] = false;
          reefAlignments[1].setBoolean(false);
          reefAlignmentsBoolean[1] = false;
          reefAlignments[2].setBoolean(false);
          reefAlignmentsBoolean[2] = false;
          reefAlignments[i].setBoolean(true);
          reefAlignmentsBoolean[i] = true;
        } else if (reefAlignments[i].getBoolean(false) != reefAlignmentsBoolean[i]) {
          reefAlignmentsBoolean[i] = false;
        }
      }

      if (climbMode.getBoolean(false)) {
        climbModeBoolean = true;
      } else {
        climbModeBoolean = false;
      }

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

    SequentialCommandGroup RaiseElevatorL3CoralCommand = new SequentialCommandGroup(
      new ZeroElevator(m_elevatorSubsystem),
      new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_3_CORAL)
    );

    SequentialCommandGroup RaiseElevatorL4CoralCommand = new SequentialCommandGroup(
      new ZeroElevator(m_elevatorSubsystem),
      new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_4_CORAL)
    );

    SequentialCommandGroup RaiseElevatorL2AlgaeCommand = new SequentialCommandGroup(
      new ZeroElevator(m_elevatorSubsystem),
      new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_2_ALGAE)
    );

    SequentialCommandGroup RaiseElevatorL3AlgaeCommand = new SequentialCommandGroup(
      new ZeroElevator(m_elevatorSubsystem),
      new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_3_ALGAE)
    );

    SequentialCommandGroup RaiseElevatorCoralStationCommand = new SequentialCommandGroup(
      new ZeroElevator(m_elevatorSubsystem),
      new SetElevator(m_elevatorSubsystem, ElevatorLevel.CORAL_STATION)
    );

    SequentialCommandGroup LowerElevatorCommand = new SequentialCommandGroup(
      new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
      new ZeroElevator(m_elevatorSubsystem)
    );

    SequentialCommandGroup RaiseAlgaeFlipperCommand = new SequentialCommandGroup(
      new ZeroAlgaeFlipper(m_algaeFlipperSubsystem),
      new SetAlgaeFlipper(m_algaeFlipperSubsystem, "up")
    );

    SequentialCommandGroup LowerAlgaeFlipperCommand = new SequentialCommandGroup(
      new SetAlgaeFlipper(m_algaeFlipperSubsystem, "down"),
      new ZeroAlgaeFlipper(m_algaeFlipperSubsystem)
    );

    SequentialCommandGroup LowerCoralFlipperLowCommand = new SequentialCommandGroup(
      new ZeroCoralFlipper(m_coralFlipperSubsystem),
      new SetCoralFlipper(m_coralFlipperSubsystem, "scoreLow")
    );

    SequentialCommandGroup LowerCoralFlipperHighCommand = new SequentialCommandGroup(
      new ZeroCoralFlipper(m_coralFlipperSubsystem),
      new SetCoralFlipper(m_coralFlipperSubsystem, "scoreHigh")
    );

    SequentialCommandGroup LowerCoralFlipperCoralStationCommand = new SequentialCommandGroup(
      new ZeroCoralFlipper(m_coralFlipperSubsystem),
      new SetCoralFlipper(m_coralFlipperSubsystem, "coralStation")
    );

    SequentialCommandGroup RaiseCoralFlipperCommand = new SequentialCommandGroup(
      new SetCoralFlipper(m_coralFlipperSubsystem, "idle"),
      new ZeroCoralFlipper(m_coralFlipperSubsystem)
    );

    ParallelCommandGroup ScoreCoralL2Command = new ParallelCommandGroup(
      Commands.sequence(
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.HOME),
        new ZeroElevator(m_elevatorSubsystem)
      ),
      Commands.sequence(
        new ZeroCoralFlipper(m_coralFlipperSubsystem),
        new SetCoralFlipper(m_coralFlipperSubsystem, "scoreLow")
      )
    );

    ParallelCommandGroup ScoreCoralL3Command = new ParallelCommandGroup(
      Commands.sequence(
        new ZeroElevator(m_elevatorSubsystem),
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_3_CORAL)
      ),
      Commands.sequence(
        new ZeroCoralFlipper(m_coralFlipperSubsystem),
        new SetCoralFlipper(m_coralFlipperSubsystem, "scoreLow")
      )
    );

    ParallelCommandGroup ScoreCoralL4Command = new ParallelCommandGroup(
      Commands.sequence(
        new ZeroElevator(m_elevatorSubsystem),
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_4_CORAL)
      ),
      Commands.sequence(
        new ZeroCoralFlipper(m_coralFlipperSubsystem),
        new SetCoralFlipper(m_coralFlipperSubsystem, "scoreHigh")
      )
    );

    SequentialCommandGroup IntakeAlgaeL2 = new SequentialCommandGroup(
      Commands.sequence(
        new ZeroElevator(m_elevatorSubsystem),
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_2_ALGAE)
      ),
      new WaitCommand(0.25),
      Commands.sequence(
        new ZeroAlgaeFlipper(m_algaeFlipperSubsystem),
        new SetAlgaeFlipper(m_algaeFlipperSubsystem, "up")
      ),
      new AlgaeIntake(m_algaeWheelsSubsystem)
    );

    SequentialCommandGroup IntakeAlgaeL3 = new SequentialCommandGroup(
      Commands.sequence(
        new ZeroElevator(m_elevatorSubsystem),
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.LEVEL_3_ALGAE)
      ),
      new WaitCommand(0.25),
      Commands.sequence(
        new ZeroAlgaeFlipper(m_algaeFlipperSubsystem),
        new SetAlgaeFlipper(m_algaeFlipperSubsystem, "up")
      ),
      new AlgaeIntake(m_algaeWheelsSubsystem)
    );

    ParallelCommandGroup IntakeCoral = new ParallelCommandGroup(
      Commands.sequence(
        new ZeroElevator(m_elevatorSubsystem),
        new SetElevator(m_elevatorSubsystem, ElevatorLevel.CORAL_STATION)
      ),
      Commands.sequence(
        new ZeroCoralFlipper(m_coralFlipperSubsystem),
        new SetCoralFlipper(m_coralFlipperSubsystem, "coralStation")
      ),
      new CoralIntake(m_coralWheelsSubsystem)
    );    

}