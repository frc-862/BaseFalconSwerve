package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.auto.AutonomousCommandFactory;
import frc.lib.pathplanner.com.pathplanner.lib.PathPoint;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.chasePiece;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    private static final Swerve drivetrain = new Swerve();
    private static final LimelightSubsystem limelight = new LimelightSubsystem();

    private static final XboxController driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

    private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(drivetrain::getPose, drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
            AutonomousConstants.DRIVE_PID_CONSTANTS, AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS, drivetrain::setModuleStates, drivetrain);
            
    public RobotContainer() {

        // Configure the trigger bindings
        configureBindings();
        configureDefaultCommands();
    }

    private void configureBindings() {
        new Trigger(driver::getXButton).whileTrue(new InstantCommand(drivetrain::park, drivetrain));
        new Trigger(driver::getAButton).whileTrue(new chasePiece(limelight, autoFactory, drivetrain));
        new Trigger(driver::getBButton).onTrue(new InstantCommand(() -> autoFactory.createManualTrajectory(AutonomousConstants.CUBE_CHASE_CONSTRAINTS,new PathPoint(drivetrain.getPose().getTranslation(), drivetrain.getYaw()), new PathPoint(new Translation2d(0, 0), new Rotation2d(0,0)))));
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), () -> (driver.getRightTriggerAxis() > 0.75)));
    }

    public Command getAutonomousCommand() { return null;}
}
