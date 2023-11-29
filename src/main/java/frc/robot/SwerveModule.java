package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.thunder.math.Conversions;
import frc.thunder.swerve.CTREModuleState;
import frc.thunder.swerve.SwerveModuleConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.DrivetrainConstants.DriveGains;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private StatusSignal<Double> anglePosition;
    private StatusSignal<Double> angleVelocity;
    private StatusSignal<Double> angleAbsolutePosition;
    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveGains.kS, DriveGains.kV, DriveGains.kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, moduleConstants.cancoderBus);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, moduleConstants.angleMotorBus);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, moduleConstants.driveMotorBus);
        configDriveMotor();

        lastAngle = getPosition().angle;

        // mAngleMotor.setRotorPosition(0);
        angleEncoder.setPosition(getCanCoderRaw());
        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getCanCoder()); 
        // SwerveModuleState.optimize(desiredState, getCanCoder());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / DrivetrainConstants.MAX_SPEED;
            mDriveMotor.setControl(new DutyCycleOut(percentOutput, true, false));
        }
        else {
            double velocity = Conversions.getInputShaftRotations((desiredState.speedMetersPerSecond / DrivetrainConstants.WHEEL_CIRCUMFERENCE), DrivetrainConstants.GEAR_RATIO);
            mDriveMotor.setControl(new VelocityDutyCycle(velocity, true, feedforward.calculate(desiredState.speedMetersPerSecond), 0, false));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DrivetrainConstants.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        // mAngleMotor.setControl(new PositionDutyCycle((angle.getDegrees() / 360) * DrivetrainConstants.ANGLE_RATIO));
        // mAngleMotor.setControl(new PositionDutyCycle((0)));
        mAngleMotor.setControl(new PositionDutyCycle(angle.getRotations()));


        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        /* Refresh all signals */
        anglePosition.refresh();
        angleVelocity.refresh();

        /* Now latency-compensate our signals */
        double angle_rot = BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity);

        return Rotation2d.fromRotations(angle_rot / DrivetrainConstants.ANGLE_RATIO);
    }

    public Rotation2d getCanCoder(){
        angleAbsolutePosition.refresh();
        return Rotation2d.fromRotations((angleAbsolutePosition.getValue()));//= - angleOffset.getRotations()));
    }

    public double getCanCoderRaw(){
        angleAbsolutePosition.refresh();
        return angleAbsolutePosition.getValue();// - angleOffset.getRotations();
    }

    public double getAngleRaw(){
        anglePosition.refresh();
        return anglePosition.getValue();
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getRotations() * DrivetrainConstants.ANGLE_RATIO;
        mAngleMotor.setRotorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        CANcoderConfiguration config = new CTREConfigs().swerveCanCoderConfig;
        config.MagnetSensor.MagnetOffset = -angleOffset.getRotations();
        angleEncoder.getConfigurator().apply(config);
        this.angleAbsolutePosition = angleEncoder.getAbsolutePosition();
        this.angleAbsolutePosition.setUpdateFrequency(250);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new CTREConfigs().swerveAngleFXConfig;
        config.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        mAngleMotor.getConfigurator().apply(config);
        this.anglePosition = mAngleMotor.getRotorPosition(); //TODO: verify whether we should be sourcing angle data from the motor or the cancoder (might not matter)
        this.angleVelocity = mAngleMotor.getRotorVelocity();
        this.anglePosition.setUpdateFrequency(250);
        this.angleVelocity.setUpdateFrequency(250);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.getConfigurator().apply(new TalonFXConfiguration());
        mDriveMotor.getConfigurator().apply(new CTREConfigs().swerveDriveFXConfig);
        this.drivePosition = mDriveMotor.getRotorPosition();
        this.driveVelocity = mDriveMotor.getRotorVelocity();
        this.drivePosition.setUpdateFrequency(250);
        this.driveVelocity.setUpdateFrequency(250);
        mDriveMotor.setRotorPosition(0);
    }

    //TODO: I think this whole function is redundant, we can just use getPosition() instead :upside_down:
    // public SwerveModuleState getState(){

    //     drivePosition.refresh();
    //     driveVelocity.refresh();

    //     //TODO: can we latency compensate the velocity? (need slope of velocity signal to do this)

    //     return new SwerveModuleState(
    //         Conversions.getOutputShaftRotations(driveVelocity.getValue(), DrivetrainConstants.GEAR_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
    //         getAngle()
    //     ); 
    // }

    public SwerveModulePosition getPosition(){
        // Refresh drive signals only, azimuth signals get refreshed in getAngle()
        drivePosition.refresh();
        driveVelocity.refresh();

        double drive_rots = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);


        return new SwerveModulePosition(
            Conversions.getOutputShaftRotations(drive_rots, DrivetrainConstants.GEAR_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
            getAngle()
        );
    }
}