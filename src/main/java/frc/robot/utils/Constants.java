// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean True = false;
    public static final boolean False = true;

    public static final class Ports {
        public static final int controller = 0;
        public static final int joystick = 1;

        public static final int frontRightDrive = 8;
        public static final int frontRightTurn = 7;
        public static final int frontRightAbsolute = 12;
        public static final int frontLeftDrive = 2;
        public static final int frontLeftTurn = 1;
        public static final int frontLeftAbsolute = 11;
        public static final int backRightDrive = 6;
        public static final int backRightTurn = 5;
        public static final int backRightAbsolute = 10;
        public static final int backLeftDrive = 4;
        public static final int backLeftTurn = 3;
        public static final int backLeftAbsolute = 9;

        public static final int cannon1 = 1;
        public static final int cannon2 = 2;
        public static final int cannon3 = 3;
        public static final int cannon4 = 4;
        public static final int cannon5 = 5;
        public static final int cannon6 = 6;

    }

    public static final class DriveConstants {
        //These will need to be in meters
        public static final double rightLeftWheels = Units.inchesToMeters(21.5);
        public static final double frontBackWheels = Units.inchesToMeters(21.5);

        public static final double FRabsoluteOffset = -1.032;
        public static final double FLabsoluteOffset = -0.370;
        public static final double BRabsoluteOffset = -0.434;
        public static final double BLabsoluteOffset = -0.357;

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(frontBackWheels / 2, rightLeftWheels / 2),//Front-Left
                new Translation2d(frontBackWheels / 2, -rightLeftWheels / 2),//Front-Right
                new Translation2d(-frontBackWheels / 2, rightLeftWheels / 2),//Back-Left
                new Translation2d(-frontBackWheels / 2, -rightLeftWheels / 2));//Back-Right
    }

    public static final class ModuleConstants {
        public static final double maxNeoSpeed = 3.68808;
        public static final double maxSpeed = 4.14528;
        public static final double maxNeoRadPerSec = 2 * 2 * Math.PI;
        public static final double wheelDiameter = 0.1016;//Units.inchesToMeters(4.0);
        public static final double driveGearRatio = 1 / 8.16;
        public static final double turnGearRatio = 1 / 12.8;
        public static final double driveEncoderToMeter = driveGearRatio * Math.PI * wheelDiameter;
        public static final double turnEncoderToRadian = turnGearRatio * 2 * Math.PI;
        public static final double driveEncoderRPMToMeterPerSec = driveEncoderToMeter / 60;
        public static final double turnEncoderRPMToRadPerSec = turnEncoderToRadian / 60;

        public static final double PTurn = 0.5;
        public static final double PDrive = 0.3;
    }
}
