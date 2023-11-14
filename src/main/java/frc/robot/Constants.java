package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.pathplanner.com.pathplanner.lib.auto.PIDConstants;


public final class Constants {

    public static final String[] FAULT_IGNORE_LIST = {};

    public static final class ControllerConstants {
        // Ports for the controllers
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int COPILOT_CONTROLLER_PORT = 1;

        // Deadband, min, and max power for the controllers
        public static final double DEADBAND = 0.1d;
        public static final double MIN_POWER = 0d;
        public static final double MAX_POWER = 1d;
    }

    public static final class DrivetrainConstants {

        //Current limits
        public static final double MAX_DRIVE_CURRENT = 60d;

        // Our drivetrain track width and Wheelbase
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(22.5);
        public static final double DRIVETRAIN_WHEELBASE_METERS =  Units.inchesToMeters(22.5);

        public static final double DRIVE_SUPPLY_LIMIT = 40d;
        public static final double DRIVE_SUPPLY_THRESHOLD = 60d;
        public static final double DRIVE_SUPPLY_TIMEOUT = 0.25;

        public static final double AZIMUTH_SUPPLY_LIMIT = 40d;
        public static final double AZIMUTH_SUPPLY_THRESHOLD = 40d;
        public static final double AZIMUTH_SUPPLY_TIMEOUT = 0.25;

        public static final double DRIVE_OPEN_RAMP_RATE = 0.25;
        public static final double DRIVE_CLOSED_RAMP_RATE = 0.0;

        public static final double MAX_SPEED = Units.feetToMeters(16.2);
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_SPEED / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        public static final double GEAR_RATIO = 6.75;
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4d);
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static final double ANGLE_RATIO = 150/7;

        public static final InvertedValue ANGLE_INVERT = InvertedValue.Clockwise_Positive;
        public static final InvertedValue DRIVE_INVERT = InvertedValue.Clockwise_Positive;

        public static final NeutralModeValue ANGLE_NEUTRAL = NeutralModeValue.Brake;
        public static final NeutralModeValue DRIVE_NEUTRAL = NeutralModeValue.Brake;

        // i swapped the left and right negations here in order to fix an issue with turn motors being oriented like an X instead of a diamond. There is likely some underlying cause that should be addressed, perhaps something to do with sensor inverts, or some other novel phoenix 6 thing
        // i do remember noticing similar behavior when using phoenix 6 on HaD, so that does point me to it being the underlying cause
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
        );

        public static final Rotation2d FRONT_LEFT_RESTING_ANGLE = Rotation2d.fromDegrees(-45d);
        public static final Rotation2d FRONT_RIGHT_RESTING_ANGLE = Rotation2d.fromDegrees(45d);
        public static final Rotation2d BACK_LEFT_RESTING_ANGLE = Rotation2d.fromDegrees(45d);
        public static final Rotation2d BACK_RIGHT_RESTING_ANGLE = Rotation2d.fromDegrees(-45d);

        // very BS tuning (no matter how well it works :P)
        public static final class AzimuthGains {
            public static final double kP = 3;//22
            public static final double kI = 0.0;//22d;
            public static final double kD = 0.1;//0005d;
            public static final double kS = 3;//54d;
        }

        // Gains vaules for PIDControllers
        public static final class DriveGains {
            public static final double kP = 0.22;// .116d;
            public static final double kI = 0.22d;
            public static final double kD = 0.001d;

            public static final double kS = 0.225;// 229d;
            public static final double kV = 0d;
            public static final double kA = 0d;
        }

        // Steer offsets for our modules
        public static final class Offsets {
            // swerve module absolute encoder offsets
                public static final double FRONT_LEFT_STEER_OFFSET  = 0.224609;
                public static final double FRONT_RIGHT_STEER_OFFSET = 0.116699;
                public static final double BACK_LEFT_STEER_OFFSET   = -0.132812;
                public static final double BACK_RIGHT_STEER_OFFSET  = -0.322754;
        }
    }

    public static final class VisionConstants {
        //This is a magic number from gridlock, may need to be changed or removed entirely
        public static final double PROCESS_LATENCY = 0.0472;
        public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0), Units.feetToMeters(26.0));
    }

    // RobotMap Constants
    public static final class RobotMap {
        // CAN IDs
        public static final class CAN {
            // Pigeon IMU ID
            public static final int PIGEON = 23;
            // Power distribution hub ID
            public static final int PDH = 21;

            // Front left CanIDs
            public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
            public static final int FRONT_LEFT_AZIMUTH_MOTOR = 2;
            public static final int FRONT_LEFT_CANCODER = 31;
            // Front right CanIDs
            public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
            public static final int FRONT_RIGHT_AZIMUTH_MOTOR = 4;
            public static final int FRONT_RIGHT_CANCODER = 33;
            // Back right CanIDs
            public static final int BACK_RIGHT_DRIVE_MOTOR = 5;
            public static final int BACK_RIGHT_AZIMUTH_MOTOR = 6;
            public static final int BACK_RIGHT_CANCODER = 32;
            // Back left CanIDs
            public static final int BACK_LEFT_DRIVE_MOTOR = 7;
            public static final int BACK_LEFT_AZIMUTH_MOTOR = 8;
            public static final int BACK_LEFT_CANCODER = 34;
        }

        public static final class BUS {
            public final static String PIGEON = "Canivore";

            public final static String DRIVE = "Canivore";

            public final static String AZIMUTH = "Canivore";

            public final static String CANCODER = "Canivore";
        }
    }

    public static final class AutonomousConstants {
        public static final PIDConstants DRIVE_PID_CONSTANTS = new PIDConstants(2.5, 0, 0); // Drive velocity PID 10.5
        public static final PIDConstants THETA_PID_CONSTANTS = new PIDConstants(4, 0, 0); // Rotation PID 7
        public static final PIDConstants POSE_PID_CONSTANTS = new PIDConstants(0, 0, 0); // X and Y position PID

        public static final PathConstraints CUBE_CHASE_CONSTRAINTS = new PathConstraints(2, 2); // TODO TEST FOR MAX 
    
        public static final double MAX_VELOCITY = 2;
        public static final double MAX_ACCELERATION = 1;
    }
    
}
