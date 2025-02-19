// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CoralExtake;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.DefaultCoralFlipperCommand;
import frc.robot.commands.DefaultCoralWheelsCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FlipCoralFlipperDown;
import frc.robot.subsystems.coral.CoralFlipperSubsystem;
import frc.robot.subsystems.coral.CoralWheelsSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralWheelsSubsystem m_coralWheelsSubsystem = new CoralWheelsSubsystem();
  private final CoralFlipperSubsystem m_coralFlipperSubsystem = new CoralFlipperSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  JoystickButton driveStart = new JoystickButton(m_driverController, Button.kStart.value);
  JoystickButton driveX = new JoystickButton(m_driverController, Button.kX.value);
  JoystickButton driveY = new JoystickButton(m_driverController, Button.kY.value);
  JoystickButton driveA = new JoystickButton(m_driverController, Button.kA.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new DefaultDriveCommand(m_robotDrive, m_driverController));
    m_coralWheelsSubsystem.setDefaultCommand(new DefaultCoralWheelsCommand(m_coralWheelsSubsystem));
    m_coralFlipperSubsystem.setDefaultCommand(new DefaultCoralFlipperCommand(m_coralFlipperSubsystem));
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

    driveX.whileTrue(
      new CoralIntake(m_coralWheelsSubsystem)
    );

    driveY.whileTrue(
      new CoralExtake(m_coralWheelsSubsystem)
    );

    driveA.toggleOnTrue(
      new FlipCoralFlipperDown(m_coralFlipperSubsystem)
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
