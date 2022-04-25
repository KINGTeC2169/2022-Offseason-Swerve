package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;

import frc.robot.utils.Constants.Ports;

import com.kauailabs.navx.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
    
    private SwerveModule frontLeft = new SwerveModule(
    Ports.frontLeftDrive,
    Ports.frontLeftTurn, 
    true, false,
    Ports.frontLeftAbsolute,
    DriveConstants.FLabsoluteOffset,
    false, false);

    private SwerveModule frontRight = new SwerveModule(
    Ports.frontRightDrive,
    Ports.frontRightTurn, 
    false, false,
    Ports.frontRightAbsolute,
    DriveConstants.FRabsoluteOffset,
    false, false);

    private SwerveModule backLeft = new SwerveModule(
    Ports.backLeftDrive,
    Ports.backLeftTurn, 
    true, false,
    Ports.backLeftAbsolute,
    DriveConstants.BLabsoluteOffset,
    false, false);

    private SwerveModule backRight = new SwerveModule(
    Ports.backRightDrive,
    Ports.backRightTurn, 
    false, false,
    Ports.backRightAbsolute,
    DriveConstants.BRabsoluteOffset,
    false, true);
    
    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem() {
        
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
        backLeft.resetEncoders();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Front Left Absolute", frontLeft.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Front Right Absolute", frontRight.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Back Left Absolute", backLeft.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Back Right Absolute", backRight.getAbsoluteTurnPosition());

        SmartDashboard.putNumber("Front Left", frontLeft.getTurnPosition());
        SmartDashboard.putNumber("Front Right", frontRight.getTurnPosition());
        SmartDashboard.putNumber("Back Left", backLeft.getTurnPosition());
        SmartDashboard.putNumber("Back Right", backRight.getTurnPosition());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.maxNeoSpeed);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void setActiveStop() {
        frontLeft.activeStop(-1);
        frontRight.activeStop(1);
        backLeft.activeStop(1);
        backRight.activeStop(-1);
    }


}
