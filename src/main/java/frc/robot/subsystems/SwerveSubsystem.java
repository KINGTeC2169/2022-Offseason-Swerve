package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utils.Constants.Ports;

public class SwerveSubsystem extends SubsystemBase {
    
    private SwerveModule frontRight = new SwerveModule(
    Ports.frontRightDrive,
    Ports.frontRightTurn, 
    Ports.frontRightAbsolute, 
    false);

    private SwerveModule frontLeft = new SwerveModule(
    Ports.frontLeftDrive,
    Ports.frontLeftTurn, 
    Ports.frontLeftAbsolute, 
    false);

    
}
