package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class LimelightTable {
    //This is a default array
    //The default value will be -2169
    private double[] arr = {-2169, -2169};
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

    //This constructor adds data to the Limelight Shuffleboard tab
    public LimelightTable(){
        tab.addNumber("TX", () -> getTX());
        tab.addNumber("TY", () -> getTY());
        tab.addNumber("TA", () -> getTA());
        tab.addNumber("TS", () -> getTS());
        tab.addBoolean("canSeeTag", () -> getTV());
        tab.addNumber("TagID", () -> getID());
    }

    public static double getTX(){
        return table.getEntry("tx").getDouble(0);
    }

    public static double getTY(){
        return table.getEntry("ty").getDouble(0);
    }

    public static double getTA(){
        return table.getEntry("ta").getDouble(0);
    }

    public static double getTS(){
        return table.getEntry("ts").getDouble(0);
    }

    public static double getID(){
        return table.getEntry("tagID").getDouble(0);
    }

    public static boolean getTV(){
        return table.getEntry("tv").getDouble(0.0) > 0;
    }
}
