package frc.robot.commands;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        final double speedCap = 0.3; //TODO: remove

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), ControllerConstants.DEADBAND) * speedCap;
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), ControllerConstants.DEADBAND) * speedCap;
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), ControllerConstants.DEADBAND) * 0.6;

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(DrivetrainConstants.MAX_SPEED), 
            rotationVal * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}