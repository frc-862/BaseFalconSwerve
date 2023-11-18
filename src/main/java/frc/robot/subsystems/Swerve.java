package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.VisionConstants;
import frc.thunder.vision.Limelight;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.swerve.SwerveModuleConstants;
import frc.thunder.util.Pose4d;
import frc.robot.Constants.DrivetrainConstants.Offsets;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDrivePoseEstimator swerveodo;
    private Field2d field = new Field2d();
    private Field2d visionField = new Field2d();
    private Field2d odoField = new Field2d();

    public Pigeon2 gyro;

    private Limelight[] limelights;

    public Swerve() {
        gyro = new Pigeon2(RobotMap.CAN.PIGEON, RobotMap.BUS.PIGEON);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        // this can be compacted significantly, but this is what you have to do to make it work with
        // our existing constants
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, new SwerveModuleConstants(RobotMap.CAN.FRONT_LEFT_DRIVE_MOTOR,
                        RobotMap.CAN.FRONT_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_LEFT_CANCODER,
                        RobotMap.BUS.DRIVE, RobotMap.BUS.AZIMUTH, RobotMap.BUS.CANCODER,
                        Rotation2d.fromRotations(Offsets.FRONT_LEFT_STEER_OFFSET))),
                new SwerveModule(1, new SwerveModuleConstants(RobotMap.CAN.FRONT_RIGHT_DRIVE_MOTOR,
                        RobotMap.CAN.FRONT_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.FRONT_RIGHT_CANCODER,
                        RobotMap.BUS.DRIVE, RobotMap.BUS.AZIMUTH, RobotMap.BUS.CANCODER,
                        Rotation2d.fromRotations(Offsets.FRONT_RIGHT_STEER_OFFSET))),
                new SwerveModule(2, new SwerveModuleConstants(RobotMap.CAN.BACK_LEFT_DRIVE_MOTOR,
                        RobotMap.CAN.BACK_LEFT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_LEFT_CANCODER,
                        RobotMap.BUS.DRIVE, RobotMap.BUS.AZIMUTH, RobotMap.BUS.CANCODER,
                        Rotation2d.fromRotations(Offsets.BACK_LEFT_STEER_OFFSET))),
                new SwerveModule(3, new SwerveModuleConstants(RobotMap.CAN.BACK_RIGHT_DRIVE_MOTOR,
                        RobotMap.CAN.BACK_RIGHT_AZIMUTH_MOTOR, RobotMap.CAN.BACK_RIGHT_CANCODER,
                        RobotMap.BUS.DRIVE, RobotMap.BUS.AZIMUTH, RobotMap.BUS.CANCODER,
                        Rotation2d.fromRotations(Offsets.BACK_RIGHT_STEER_OFFSET)))};

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug with inverting
         * motors. See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        // resetModulesToAbsolute();

        this.limelights = new Limelight[] {
            new Limelight("limelight-back", "10.8.62.11"),
            new Limelight("limelight-front", "10.8.62.12")
        };
        // limelight.setCameraPoseRobotSpace(new Pose3d(Units.inchesToMeters(3.375), 0, Units.inchesToMeters(21.6), new Rotation3d(0, 0, 0)));
        this.poseEstimator = new SwerveDrivePoseEstimator(DrivetrainConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions(), limelights[0].getAlliancePose().toPose2d());
        this.poseEstimator.update(getYaw(), getModulePositions());
        this.swerveodo = new SwerveDrivePoseEstimator(DrivetrainConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions(), new Pose2d());
        swerveodo.resetPosition(getYaw(), getModulePositions(), limelights[0].getAlliancePose().toPose2d());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative,
            boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                DrivetrainConstants.SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(),
                                translation.getY(), -rotation, getYaw())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                DrivetrainConstants.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void park() {
        // as of yet, non-functional
        mSwerveMods[0].setDesiredState(
                new SwerveModuleState(0d, DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE), true);
        mSwerveMods[1].setDesiredState(
                new SwerveModuleState(0d, DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE), true);
        mSwerveMods[2].setDesiredState(
                new SwerveModuleState(0d, DrivetrainConstants.BACK_LEFT_RESTING_ANGLE), true);
        mSwerveMods[3].setDesiredState(
                new SwerveModuleState(0d, DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE), true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * Get Pose estimator pose
     * @return pose2d (x,y)
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * 
     * @return Pose estimator pose without vision
     */
    public Pose2d getPoseNoVision() {
        return swerveodo.getEstimatedPosition();
    }



    /**
     * Sets poseEstimator position to given: (x, y), and gyro heading
     * @param pose pose2d, (x, y)
     */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
        swerveodo.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    /**
     * 
     * @return Rotation 2d from gyro
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    /**
     * Gets the kinematics of the robot.
     * 
     * @return the kinematics of the robot
     */
    public SwerveDriveKinematics getDriveKinematics() {
        return DrivetrainConstants.SWERVE_KINEMATICS;
    }

    @Override
    public void periodic() {
        // Update pose Estimator
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
        swerveodo.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
        for (Limelight limelight : Limelight.filterLimelights(limelights)) {
            Pose4d pose = limelight.getAlliancePose();
            poseEstimator.addVisionMeasurement(pose.toPose2d(), Timer.getFPGATimestamp() - Units.millisecondsToSeconds(pose.getLatency()) - VisionConstants.PROCESS_LATENCY);
            visionField.setRobotPose(pose.getX(), pose.getY(), pose.getRotation().toRotation2d());
        }
        field.setRobotPose(getPose());
        odoField.setRobotPose(swerveodo.getEstimatedPosition());

        LightningShuffleboard.set("Odometry", "Pose Estimator Field", field);
        LightningShuffleboard.set("Odometry", "Vision Field", visionField);
        LightningShuffleboard.set("Odometry", "Odometry Field", odoField);

        LightningShuffleboard.setDouble("Swerve", "yaw", getYaw().getDegrees());
    }
}
