/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        //drivetrain CAN IDs
        public static final int kFrontLeftMotorPort = 3;
        public static final int kRearLeftMotorPort = 4;
        public static final int kFrontRightMotorPort = 1;
        public static final int kRearRightMotorPort = 2;
        //Shooter CAN IDs
        public static final int kTopShooter = 5;
        public static final int kBottomShooter = 6;
        //Climb Elevator CAN ID
        public static final int kElevatorCAN = 7;
        // Indexer CAN input
        public static final int kMidIndexMotor = 8;
        public static final int kBackIndexMotor = 9;
        // Climb PWM input
        public static final int kClimbMotor = 1;
        //ball intake PWM
        public static final int kIntakeMotor = 0;
        //reverse motor booleans
        public static final boolean kClimberIsReversed = false;
        public static final boolean kElevatorIsReversed = false;
        public static final boolean kIntakeIsReversed = false;
        //reverse gyro readings
        public static final boolean kGyroReversed = false;
        //pid constants for turn to target
        public static final double kF = 0.0;
        public static final double kP = 0.085;
        public static final double kI = 0.045;
        public static final double kD = 0.003;
        public static final double kTolerance = 0.1;

        //gear ratio of drive train
        public static final double driveRatio = 8.45;
        public static final boolean rightEncoderisReversed = false;
        public static final boolean leftEncoderisReversed = true;

         //target position for elevator
         public static double kSetPositionClimb = 20000;
         //soft limit in raw units for climb elevtor
         public static final int kForwardSoftLimit = 22000;
         public static final int kReverseSoftLimit = 0;
         //for deviding encoder values into useful units
         public static final double kEncoderRevolutions = 1;
    }

    public static final class toAngleConstants{
        //constants for turn to angle
        public static final double kTurnToleranceDeg = 10.0;
        public static final double kTurnRateToleranceDegPerS = 10.0;
        public static final double kP = 0.002;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class PIDtoBall {
        public static final double kP = 0.03;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        //TODO: test guitar then delete second op controller
        public static final int kGuitarPort = 2;
        public static final int kFrontCamera = 0;
        public static final int kRearCamera = 1;

    }

    public static final class pdpConstants {
        public static final int kTopShooterMotor = 1;
        public static final int kBottomShooterMotor = 2;
    }

    public static final class AutoConstants{
        public static final double ksVolts = 0.108;
        public static final double kvVoltSecondsPerMeter = 1.95;
        public static final double kaVoltSecondsSquaredPerMeter = 0.249;

        // Example value only - as above, this must be tuned for your drive!
        //TODO: where does kPDriveVel come from in the data?
        public static final double kPDriveVel = 5.5;

        //distance between wheels in meters
        public static final double kTrackwidthMeters = 0.73914;

        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class swagGuitar {
        //frets colored-buttons TODO: map buttons actual IDs
        public static final int greenButton = 4;
        public static final int redButton = 2;
        public static final int yellowButton = 0;
        public static final int blueButton = 1;
        public static final int orangeButton = 1;
        //bar that you can pull down
        public static final int swagBar = 1;
        //buttom plus buttons
        public static final int plusButton = 1;
        public static final int minusButton = 1;
        //strummer is 0 and 180 degrees d-pad
        public static final int strumUp = 0;
        public static final int strumDown = 180;
        //joystick axises
        public static final int xAxis = 1;
        public static final int yAxis = 2;
    }    

}
