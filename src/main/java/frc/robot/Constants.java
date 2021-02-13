// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int CAN_ID_MOTOR_DRIVETRAIN_FRONT_LEFT  = 4;  //2
    public static final int CAN_ID_MOTOR_DRIVETRAIN_FRONT_RIGHT = 3;  //1
    public static final int CAN_ID_MOTOR_DRIVETRAIN_BACK_LEFT   = 1;  //3
    public static final int CAN_ID_MOTOR_DRIVETRAIN_BACK_RIGHT  = 2;  //4

    public static final int JOYSTICK_ID_DRIVER = 0;
    public static final int JOYSTICK_ID_OPERATOR = 1;

    public static final int ENCODER_ID_DRIVETRAIN_FRONT_RIGHT_A = 0;
    public static final int ENCODER_ID_DRIVETRAIN_FRONT_RIGHT_B = 1;
    public static final int ENCODER_ID_DRIVETRAIN_FRONT_LEFT_A = 2;
    public static final int ENCODER_ID_DRIVETRAIN_FRONT_LEFT_B = 3;
    public static final int ENCODER_ID_DRIVETRAIN_BACK_RIGHT_A = 6;
    public static final int ENCODER_ID_DRIVETRAIN_BACK_RIGHT_B = 7;
    public static final int ENCODER_ID_DRIVETRAIN_BACK_LEFT_A = 4;
    public static final int ENCODER_ID_DRIVETRAIN_BACK_LEFT_B = 5;
    public static final double ENCODER_WHEEL_DIAMETER = 6.0;
    public static final double ENCODER_PULSE_PER_REVOLUTION = 250.0;
    public static final double ENCODER_DISTANCE_PER_PULSE = (double)(Math.PI*ENCODER_WHEEL_DIAMETER)/ENCODER_PULSE_PER_REVOLUTION;
    public static final double ENCODER_MAX_PERIOD = 0.1;
    public static final int ENCODER_SAMPLES_TO_AVERAGE = 5;
    public static final int ENCODER_MIN_RATE = 10;

    public static final class DriveConstants {

		public static final double ksVolts = 0.22;
		public static final double kvVoltSecondsPerMeter = 1.98;
		public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 3.5;
    public static final double trackWidthMeters = 0.61;
		public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidthMeters);
        
    }

    public class AutoConstants {
        public final static double kMaxSpeedMetersPerSecond = 1.7;
        public final static double kMaxAccelerationMetersPerSecondSquared = 2.0;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = .07;
        
    }


}
