package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utils.Constants.DriveConstants;

public class NavX {

    private ShuffleboardTab tab = Shuffleboard.getTab("NavX");

    public NavX(){
        tab.addNumber("Pitch", () -> getX());
        tab.addNumber("Roll", () -> getY());
        tab.addNumber("Yaw", () -> getZ());
        tab.addNumber("Angle", () -> getAngle());
    }

    public static double getX(){
        return DriveConstants.gyro.getPitch();
    }

    public static double getY(){
        return DriveConstants.gyro.getRoll();
    }

    public static double getZ(){
        return DriveConstants.gyro.getYaw();
    }

    public static double getAngle(){
        return DriveConstants.gyro.getAngle();
    }

    public static Rotation2d getCurrentRotation(){
        return DriveConstants.gyro.getRotation2d();
    }
    
}
