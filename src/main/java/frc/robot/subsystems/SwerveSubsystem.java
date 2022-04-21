package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utils.Constants.Ports;

import com.kauailabs.navx.frc.AHRS;

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

    private SwerveModule backRight = new SwerveModule(
    Ports.backRightDrive,
    Ports.backRightTurn, 
    Ports.backRightAbsolute, 
    false);
    
    private SwerveModule backLeft = new SwerveModule(
    Ports.backLeftDrive,
    Ports.backLeftTurn, 
    Ports.backLeftAbsolute, 
    false);

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 5);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }


}
