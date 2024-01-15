package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utils.Constants.Ports;

public class NavX {

    private ShuffleboardTab tab = Shuffleboard.getTab("NavX");

    public NavX(){
        tab.addNumber("Pitch", () -> getX());
        tab.addNumber("Roll", () -> getY());
        tab.addNumber("Yaw", () -> getZ());
        tab.addNumber("Angle", () -> getAngle());
    }

    public static double getX(){
        return Ports.gyro.getPitch();
    }

    public static double getY(){
        return Ports.gyro.getRoll();
    }

    public static double getZ(){
        return Ports.gyro.getYaw();
    }

    public static double getAngle(){
        return Ports.gyro.getAngle();
    }

    public static Rotation2d getCurrentRotation(){
        return Ports.gyro.getRotation2d();
    }
    
}
