package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Limelight {
    //This is a default array
    //The default value will be -2169
    private double[] arr = {-2169, -2169};
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    private ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

    //This constructor adds data to the Limelight Shuffleboard tab
    public Limelight(){
        tab.addNumber("TX", () -> getTX());
        tab.addNumber("TY", () -> getTY());
        tab.addNumber("TA", () -> getTA());
        tab.addNumber("TS", () -> getTS());
    }

    public static double getTX(){
        return table.getEntry("tx").getDouble(-2169);
    }

    public static double getTY(){
        return table.getEntry("ty").getDouble(-2169);
    }

    public static double getTA(){
        return table.getEntry("ta").getDouble(-2169);
    }

    public static double getTS(){
        return table.getEntry("ts").getDouble(-2169);
    }

}
