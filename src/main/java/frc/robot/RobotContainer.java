// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.algae.*;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AutoSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_vision);
  private final AutoSubsystem m_auto = new AutoSubsystem(m_robotDrive);

  public static final Field2d field = new Field2d();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  JoystickButton driveX = new JoystickButton(m_driverController, Button.kX.value);
  JoystickButton driveY = new JoystickButton(m_driverController, Button.kY.value);
  JoystickButton driveA = new JoystickButton(m_driverController, Button.kA.value);
  JoystickButton driveB = new JoystickButton(m_driverController, Button.kB.value);
  JoystickButton driveStart = new JoystickButton(m_driverController, Button.kStart.value);
  POVButton driveUpDPad = new POVButton(m_driverController, 0);
  POVButton driveRightDPad = new POVButton(m_driverController, 90);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new DefaultDriveCommand(m_robotDrive, m_driverController));
    m_elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(m_elevatorSubsystem));
    m_algaeFlipperSubsystem.setDefaultCommand(new DefaultAlgaeFlipperCommand(m_algaeFlipperSubsystem));
    m_algaeWheelsSubsystem.setDefaultCommand(new DefaultAlgaeWheelsCommand(m_algaeWheelsSubsystem));
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

    driveX.toggleOnTrue(
      new MoveElevatorToSetpoint(m_elevatorSubsystem)
    );

    driveY.toggleOnTrue(
      new FlipFlipperUp(m_algaeFlipperSubsystem)
    );

    driveA.whileTrue(
      new AlgaeIntake(m_algaeWheelsSubsystem)
    );

    driveB.whileTrue(
      new AlgaeExtake(m_algaeWheelsSubsystem)
    );

    driveUpDPad.whileTrue(
      m_robotDrive.pathFindToProcessor()
    );

    driveRightDPad.whileTrue(
      m_robotDrive.pathFindToCoralStationRight()
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
}
