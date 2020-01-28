/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
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
        //turret CAN ID
        public static final int kTurretMotor = 7;
        // Climb PWM input
        public static final int kClimbMotor = 0;
        public static final int kElevator = 1;
        // Indexer PWM input
        public static final int kIndexRoller = 2;
        public static final int kMidIndexMotor = 3;
        public static final int kBackIndexMotor = 4;
        //ball intake PWM
        public static final int kIntakeMotor = 5;
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

    }

    public static final class toAngleConstants{
        //constants for turn to angle
        public static final double kTurnToleranceDeg = 5.0;
        public static final double kTurnRateToleranceDegPerS = 10.0;
        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class PIDtoBall {
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;
        public static final int kOperatorControllerPort = 0;
        public static final int kFrontCamera = 0;
        public static final int kRearCamera = 1;

    }

    public static final class pdpConstants {
        public static final int kTopShooterMotor = 1;
        public static final int kBottomShooterMotor = 2;
    }

}
