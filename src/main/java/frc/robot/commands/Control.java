package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;

public class Control {
    private static XboxController controller = new XboxController(1);

    //Controller inputs
    public static double getLeftControllerX() {
        return controller.getLeftX();
    }

    public static double getLeftControllerY() {
        return controller.getLeftY();
    }

    public static double getRightControllerX() {
        return controller.getRightX();
    }

    public static double getRightControllerY() {
        return controller.getRightY();
    }

    public static boolean getControllerA() {
        return controller.getAButton();
    }
    public static boolean getControllerAPressed() {
        return controller.getAButtonPressed();
    }

    public static boolean getControllerB() {
        return controller.getBButton();
    }
    public static boolean getControllerBPressed() {
        return controller.getBButtonPressed();
    }

    public static boolean getControllerX() {
        return controller.getXButton();
    }
    public static boolean getControllerXPressed() {
        return controller.getXButtonPressed();
    }

    public static boolean getControllerY() {
        return controller.getYButton();
    }
    public static boolean getControllerYPressed() {
        return controller.getYButtonPressed();
    }

    public static double getRightControllerTrigger() {
        return controller.getRightTriggerAxis();
    }

    public static double getLeftControllerTrigger() {
        return controller.getLeftTriggerAxis();
    }

    public static boolean getLeftControllerBumper() {
        return controller.getLeftBumper();
    }
    public static boolean getLeftControllerBumperPressed() {
        return controller.getLeftBumperPressed();
    }

    public static boolean getRightControllerBumper() {
        return controller.getRightBumper();
    }
    public static boolean getRightControllerBumperPressed() {
        return controller.getRightBumperPressed();
    }

    public static boolean getLeftControllerStick() {
        return controller.getLeftStickButtonPressed();
    }

    public static boolean getRightControllerStick() {
        return controller.getRightStickButtonPressed();
    }

    public static int getDPad() {
        return controller.getPOV();
    }

    public static boolean babyBackRibs() {
        return controller.getBackButton();
    }

    public static boolean startYourEngines() {
        return controller.getStartButtonPressed();
    }

}