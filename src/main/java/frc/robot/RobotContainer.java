// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FollowAprilTagCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.LimelightTable;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants.Ports;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //These constructors display data to shuffleboard
  private final NavX navx = new NavX();
  private final LimelightTable limelight = new LimelightTable();

  //Follow AprilTag command
  private Command followAprilTag;

  //private final Joystick driverJoystick = new Joystick(Ports.joystick);
  private final XboxController controller = new XboxController(Ports.controller);
  //private final CommandJoystick moveStick = new CommandJoystick(Ports.moveStick);
  //private final CommandJoystick turnStick = new CommandJoystick(Ports.turnStick);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
          swerveSubsystem,
          () -> controller.getLeftY(),
          () -> controller.getLeftX(),
          () -> controller.getRightX(),
          () -> !controller.getAButtonPressed(),
          () -> controller.getStartButtonPressed(),
          () -> controller.getYButton(),
          () -> controller.getBButton(),
          () -> controller.getAButton(),
          () -> controller.getXButton(),
          () -> controller.getLeftBumper(),
          () -> controller.getRightBumper())
          );

    configureButtonBindings();

    followAprilTag = new FollowAprilTagCmd(swerveSubsystem);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(controller, 7).onTrue(swerveSubsystem.zeroHeading());
    // new JoystickButton(controller, 8).onTrue(swerveSubsystem.resetEncoders());
    //new JoystickButton(controller, 2).whileActiveContinuous(() -> swerveSubsystem.setActiveStop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return followAprilTag;
  }
}
