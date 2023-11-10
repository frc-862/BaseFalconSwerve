package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.thunder.math.Conversions;
import frc.thunder.swerve.CTREModuleState;
import frc.thunder.swerve.SwerveModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
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
    private AnalogEncoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveGains.kS, DriveGains.kV, DriveGains.kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new AnalogEncoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        REVConfigs.configAngleMotor(mAngleMotor);

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        REVConfigs.configDriveMotor(mDriveMotor);

        lastAngle = getState().angle;

        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getAbsPos());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / DrivetrainConstants.MAX_SPEED;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.getInputShaftRotations((desiredState.speedMetersPerSecond / DrivetrainConstants.WHEEL_CIRCUMFERENCE), DrivetrainConstants.DRIVE_RATIO);
            mDriveMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DrivetrainConstants.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.getPIDController().setReference(Conversions.getInputShaftRotations(angle.getRotations(), DrivetrainConstants.ANGLE_RATIO), ControlType.kPosition);

        lastAngle = angle;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(MathUtil.inputModulus(Conversions.getOutputShaftRotations(mAngleMotor.getEncoder().getPosition(), DrivetrainConstants.ANGLE_RATIO), 0, 1));
    }

    public Rotation2d getAbsPos(){
        return Rotation2d.fromRotations((MathUtil.inputModulus(angleEncoder.getAbsolutePosition(), 0, 1)));
    }

    public double getAbsRaw(){
        return angleEncoder.getAbsolutePosition();
    }

    public double getAngleRaw(){
        return mAngleMotor.getEncoder().getPosition();
    }

    public void resetToAbsolute(){
        double absolutePosition = getAbsPos().getRotations();
        mAngleMotor.getEncoder().setPosition(Conversions.getInputShaftRotations(absolutePosition, DrivetrainConstants.ANGLE_RATIO));
    }

    private void configAngleEncoder(){
        angleEncoder.setPositionOffset(-angleOffset.getRotations());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.getOutputShaftRotations(mDriveMotor.getEncoder().getVelocity(), DrivetrainConstants.DRIVE_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.getOutputShaftRotations(mDriveMotor.getEncoder().getPosition(), DrivetrainConstants.DRIVE_RATIO) * DrivetrainConstants.WHEEL_CIRCUMFERENCE,
            getAngle()
        );
    }
}