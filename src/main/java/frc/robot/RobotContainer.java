package frc.robot;

import java.util.HashMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.thunder.auto.Autonomous;
import frc.thunder.auto.AutonomousCommandFactory;
import frc.thunder.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.thunder.pathplanner.com.pathplanner.lib.PathPoint;
import frc.thunder.vision.Limelight;
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

    private static final XboxController driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

     private static final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(drivetrain::getPoseNoVision, drivetrain::resetOdometry, drivetrain.getDriveKinematics(),
            AutonomousConstants.DRIVE_PID_CONSTANTS, AutonomousConstants.THETA_PID_CONSTANTS, AutonomousConstants.POSE_PID_CONSTANTS, drivetrain::setModuleStates, drivetrain);

    @Override
    protected void configureButtonBindings() {
        // new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetModulesToAbsolute));
        new Trigger(driver::getXButton).whileTrue(new InstantCommand(drivetrain::park, drivetrain));
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), () -> (driver.getRightTriggerAxis() > 0.75)));
    }

    @Override
    protected void configureAutonomousCommands(){
        autoFactory.makeTrajectory("1-Meter-Back", new HashMap<>(), 
            new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("1-Meter-Forward", new HashMap<>(), 
            new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("1-Meter-Left", new HashMap<>(), 
            new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("1-Meter-Right", new HashMap<>(), 
            new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        autoFactory.makeTrajectory("Square", new HashMap<>(), 
            new PathConstraints(AutonomousConstants.MAX_VELOCITY, AutonomousConstants.MAX_ACCELERATION));
        Autonomous.register("FORCE", new InstantCommand(() -> drivetrain.drive(new Translation2d(0, 0), 0, true, false)));
    }

    protected AutonomousCommandFactory getCommandFactory(){
        return autoFactory;
    }
    
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
