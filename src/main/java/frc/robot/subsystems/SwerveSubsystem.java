package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;
import static frc.robot.utils.Constants.ModuleConstants.*;

import frc.robot.utils.Constants.Ports;

import com.kauailabs.navx.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
    
    //Creates an instance of SwerveModule for each module on the robot
    private SwerveModule frontLeft = new SwerveModule(
    Ports.frontLeftDrive,
    Ports.frontLeftTurn, 
    true, false,
    Ports.frontLeftAbsolute,
    DriveConstants.FLabsoluteOffset,
    false);

    private SwerveModule frontRight = new SwerveModule(
    Ports.frontRightDrive,
    Ports.frontRightTurn, 
    false, false,
    Ports.frontRightAbsolute,
    DriveConstants.FRabsoluteOffset,
    false);

    private SwerveModule backLeft = new SwerveModule(
    Ports.backLeftDrive,
    Ports.backLeftTurn, 
    true, false,
    Ports.backLeftAbsolute,
    DriveConstants.BLabsoluteOffset,
    false);

    private SwerveModule backRight = new SwerveModule(
    Ports.backRightDrive,
    Ports.backRightTurn, 
    false, false,
    Ports.backRightAbsolute,
    DriveConstants.BRabsoluteOffset,
    false);
    
    //Creates an AHRS object to get inputs from the navX
    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem() {
        
        //Creates a new thread, which sleeps and then zeros out the gyro
        //Uses a new thread so that it doesn't pause all other code running
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        
    }

    public void zeroHeading() {
        System.out.println("Zeroing gyro \n.\n.\n.\n.\n.\n.\n.");
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
        //return Rotation2d.fromDegrees(getHeading());
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
        backLeft.resetEncoders();
    }

    @Override
    public void periodic() {
        //Runs during robot periodic, displays shuffleboard data for this subsystem
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

    /**Takes an array of SwerveModuleStates and sets each SwerveModule to its respective state */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxNeoSpeed);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /**Puts wheels in 'X' position and sets driving to a velocity-PID loop set at 0m/s */
    public void setActiveStop() {
        System.out.println("1\n1\n1\n1\n1\n1\n1\n1");
        frontLeft.activeStop(-1);
        frontRight.activeStop(1);
        backLeft.activeStop(1);
        backRight.activeStop(-1);
    }


}
