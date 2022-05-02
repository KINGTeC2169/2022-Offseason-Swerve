package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
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

  private final Joystick driverJoystick = new Joystick(Ports.joystick);
  private final XboxController controller = new XboxController(Ports.controller);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
          swerveSubsystem,
          //() -> controller.getLeftY(),
          //() -> controller.getLeftX(),
          () -> controller.getRightX(),
          () -> controller.getRawButton(1),
          () -> controller.getRawButton(2),
          () -> controller.getRawButton(3),
          () -> controller.getRawButton(4),
          () -> controller.getRawButton(5),
          () -> controller.getPOV()
          ));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //BUTTON NUMBERS START AT 1
    new JoystickButton(controller, 7).whenPressed(() -> swerveSubsystem.resetEncoders());
    new JoystickButton(controller, 8).whenPressed(() -> swerveSubsystem.zeroHeading());
    //new JoystickButton(controller, 9).whenPressed(() -> swerveSubsystem.setActiveStop());
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
