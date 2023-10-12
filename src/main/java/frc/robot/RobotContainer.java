package frc.robot;

import java.util.HashMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.auto.AutonomousCommandFactory;
import frc.lib.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.lib.pathplanner.com.pathplanner.lib.PathPoint;
import frc.lib.util.Limelight;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.FaultMonitor;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    private static final Swerve drivetrain = new Swerve();
    private static final Limelight limelight = new Limelight("limelight");
    private static final LimelightSubsystem limelightSub = new LimelightSubsystem();
    private static final FaultMonitor FaultMonitor = new FaultMonitor();

    

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
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), () -> (driver.getRightTriggerAxis() > 0.75)));
    }

    private void configureAutonomousCommands(){
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
        
    }

    protected AutonomousCommandFactory getCommandFactory(){
        return autoFactory;
    }

    public Command getAutonomousCommand() { return null;}
}
