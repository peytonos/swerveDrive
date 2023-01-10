package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_GEAR_RATIO = 1 / 5.8462;
        public static final double TURNING_GEAR_RATIO = 1 / 18.0;
        public static final double DRIVE_ENCODER_ROTATIONS_TO_METERS = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_ENCODER_ROTATIONS_TO_RADIANS = TURNING_GEAR_RATIO * 2 * Math.PI;
        public static final double DRIVE_ENCODER_RPMS_TO_METERS_PER_SECOND = DRIVE_ENCODER_ROTATIONS_TO_METERS / 60;
        public static final double TURNING_ENCODER_RPMS_TO_RADIANS_PER_SECOND = TURNING_ENCODER_ROTATIONS_TO_RADIANS / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double TRACK_WIDTH = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(25.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2));

        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 8;
        public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 4;

        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 7;
        public static final int BACK_LEFT_TURNING_MOTOR_PORT = 1;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 5;
        public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 3;

        public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = true;
        public static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = true;
        public static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = true;
        public static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = true;

        public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = true;
        public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = true;
        public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false;

        public static final int FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 0;
        public static final int BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 2;
        public static final int FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 1;
        public static final int BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 3;

        public static final boolean FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHTT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

        public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS = -0.254;
        public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RADIANS = -1.252;
        public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = -1.816;
        public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RADIANS = -4.811;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 5;
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;

        public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;
        public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = //
            PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final int FRONT_LEFT_DRIVE_ENCODER_NUMBER = 1;
        public static final int BACK_LEFT_DRIVE_ENCODER_NUMBER = 2;
        public static final int FRONT_RIGHT_DRIVE_ENCODER_NUMBER = 3;
        public static final int BACK_RIGHT_DRIVE_ENCODER_NUMBER = 4;

        public static final int FRONT_LEFT_TURNING_ENCODER_NUMBER = 5;
        public static final int BACK_LEFT_TURNING_ENCODER_NUMBER = 6;
        public static final int FRONT_RIGHT_TURNING_ENCODER_NUMBER = 7;
        public static final int BACK_RIGHT_TURNING_ENCODER_NUMBER = 8;


    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }
}
