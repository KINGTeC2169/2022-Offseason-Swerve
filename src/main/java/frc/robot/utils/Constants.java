// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Ports {
        public static final int frontRightDrive = 0;
        public static final int frontRightTurn = 0;
        public static final int frontRightAbsolute = 0;
        public static final int frontLeftDrive = 0;
        public static final int frontLeftTurn = 0;
        public static final int frontLeftAbsolute = 0;
        public static final int backRightDrive = 0;
        public static final int backRightTurn = 0;
        public static final int backRightAbsolute = 0;
        public static final int backLeftDrive = 0;
        public static final int backLeftTurn = 0;
        public static final int backLeftAbsolute = 0;
    }

    public static final class ModuleConstants {
        public static final double wheelDiameter = 0;
        public static final double driveGearRatio = 0;
        public static final double turnGearRatio = 0;
        public static final double driveEncoderToMeter = driveGearRatio * Math.PI * wheelDiameter;
        public static final double turnEncoderToRadian = turnGearRatio * 2 * Math.PI;
        public static final double driveEncoderRPMToMeterPerSec = driveEncoderToMeter / 60;
        public static final double turnEncoderRPMToRadPerSec = turnEncoderToRadian / 60;
        public static final double PTurn = 0.5;
    }
}
