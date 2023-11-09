package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import frc.thunder.auto.AutonomousCommandFactory;

public class RobotContainer extends LightningContainer {
    private static final Swerve drivetrain = new Swerve();
    // private static final LimelightSubsystem limelight = new LimelightSubsystem();

    private static final XboxController driver =
            new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

    private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(
            drivetrain::getPose, drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
            AutonomousConstants.DRIVE_PID_CONSTANTS, AutonomousConstants.THETA_PID_CONSTANTS,
            AutonomousConstants.POSE_PID_CONSTANTS, drivetrain::setModuleStates, drivetrain);

    @Override
    protected void configureButtonBindings() {
        // new Trigger(driver::getAButton).onTrue(new
        // InstantCommand(drivetrain::resetModulesToAbsolute));
        new Trigger(driver::getXButton).whileTrue(new InstantCommand(drivetrain::park, drivetrain));

    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
                new TeleopSwerve(drivetrain, () -> driver.getLeftY(), () -> driver.getLeftX(),
                        () -> driver.getRightX(), () -> (driver.getRightTriggerAxis() > 0.75)));
    }

    @Override
    protected void configureAutonomousCommands() {}

    @Override
    protected void releaseDefaultCommands() {}

    @Override
    protected void initializeDashboardCommands() {}

    @Override
    protected void configureFaultCodes() {}

    @Override
    protected void configureFaultMonitors() {}

    @Override
    protected void configureSystemTests() {}
}
