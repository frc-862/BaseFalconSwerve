package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.thunder.math.Conversions;
import frc.thunder.swerve.CTREModuleState;
import frc.thunder.swerve.SwerveModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.DrivetrainConstants.DriveGains;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveGains.kS, DriveGains.kV, DriveGains.kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, moduleConstants.cancoderBus);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        REVConfigs angleConfig = new REVConfigs();
        angleConfig.configAngleMotor(mAngleMotor);

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        REVConfigs driveConfig = new REVConfigs();
        driveConfig.configDriveMotor(mDriveMotor);

        lastAngle = getState().angle;

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
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.getInputShaftRotations((desiredState.speedMetersPerSecond / DrivetrainConstants.WHEEL_CIRCUMFERENCE), DrivetrainConstants.GEAR_RATIO);
            mDriveMotor.set(velocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DrivetrainConstants.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        

        mAngleMotor.set(angle.getRotations());


        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromRotations(mAngleMotor.getEncoder().getPosition() / DrivetrainConstants.ANGLE_RATIO);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations((angleEncoder.getAbsolutePosition().getValue()));//= - angleOffset.getRotations()));
    }

    public double getCanCoderRaw(){
        return angleEncoder.getAbsolutePosition().getValue();// - angleOffset.getRotations();
    }

    public double getAngleRaw(){
        return mAngleMotor.getEncoder().getPosition();
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getRotations() * DrivetrainConstants.ANGLE_RATIO;
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        CANcoderConfiguration config = new CTREConfigs().swerveCanCoderConfig;
        config.MagnetSensor.MagnetOffset = -angleOffset.getRotations();
        angleEncoder.getConfigurator().apply(config);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.getOutputShaftRotations(mDriveMotor.getEncoder().getVelocity(), DrivetrainConstants.GEAR_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.getOutputShaftRotations(mDriveMotor.getEncoder().getPosition(), DrivetrainConstants.GEAR_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
            getAngle()
        );
    }
}