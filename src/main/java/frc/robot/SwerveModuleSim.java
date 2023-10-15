package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.thunder.math.Conversions;
import frc.thunder.swerve.CTREModuleState;
import frc.thunder.swerve.SwerveModuleConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.DrivetrainConstants.DriveGains;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModuleSim extends SwerveModule {
    public int moduleNumber;

    // yes i realize theyre neos. i dont wanna talk about it.
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004096955);

    private final PIDController driveFeedback = new PIDController(0.9, 0.0, 0.0);
    private final PIDController turnFeedback = new PIDController(23.0, 0.0, 0.0);
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.116970, 0.133240, 0.0);

    private double turnPos = Math.random() * 2.0 * Math.PI;
    private double drivePos = 0.0;
    private double driveVeloc = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private final double loopTime = 0.02;

    private final int driveInvert = 1;
    private final int turnInvert = -1;


    public SwerveModuleSim(int moduleNumber, SwerveModuleConstants moduleConstants){
        super(moduleNumber, moduleConstants);
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void updateStates() {
        // Convert flywheel velocity sim numbers to positional numbers
        driveSim.update(loopTime);
        turnSim.update(loopTime);
    
        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * loopTime;
        turnPos += angleDiffRad;
        while (turnPos < 0) {
          turnPos += 2.0 * Math.PI;
        }
        while (turnPos > 2.0 * Math.PI) {
          turnPos -= 2.0 * Math.PI;
        }
    
        drivePos -= (driveSim.getAngularVelocityRadPerSec() * loopTime);
        driveVeloc =  driveSim.getAngularVelocityRadPerSec();
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getCanCoder()); 
        SwerveModuleState.optimize(desiredState, getCanCoder());

        // its assumed that this is called every loop, so we can just update the states here
        updateStates();

        setAngle(desiredState);
        setSpeed(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState){
        // since there is no built-in PID, we also have to run the PID loop ourselves
        setDriveVoltage(driveFeedback.calculate(driveVeloc, desiredState.speedMetersPerSecond) + driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }

    private void setAngle(SwerveModuleState desiredState){
        // since there is no built-in PID, we also have to run the PID loop ourselves
        setTurnVoltage(turnFeedback.calculate(turnPos, desiredState.angle.getRadians()));
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromRadians(turnPos);
    }

    // I apologize for this, but theres no way I'm implementing a CANcoder for a simulation
    public Rotation2d getCanCoder(){
        return getAngle();
    }

    public double getCanCoderRaw(){
        return getAngle().getDegrees();
    }

    public double getAngleRaw(){
        return getAngle().getDegrees();
    }

    public void resetToAbsolute(){
        // :P
    }


    public SwerveModuleState getState(){
        return new SwerveModuleState(
            // Not 100% sure if conversion is necessary for sim
            // Conversions.getOutputShaftRotations(driveVeloc, DrivetrainConstants.GEAR_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
            driveVeloc,
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            // Not 100% sure if conversion is necessary for sim
            // Conversions.getOutputShaftRotations(mDriveMotor.getRotorPosition().getValue(), DrivetrainConstants.GEAR_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
            drivePos,
            getAngle()
        );
    }

    private void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveInvert * driveAppliedVolts);
    }

    private void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }
}