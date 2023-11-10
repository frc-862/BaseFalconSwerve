package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.AzimuthGains;
import frc.robot.Constants.DrivetrainConstants.DriveGains;

public class REVConfigs {
    public static CANSparkMax configAngleMotor(CANSparkMax motor) {
        motor.setInverted(DrivetrainConstants.ANGLE_INVERT);
        motor.setSmartCurrentLimit(DrivetrainConstants.AZIMUTH_SUPPLY_LIMIT);
        motor.enableVoltageCompensation(DrivetrainConstants.AZIMUTH_VOLTAGE_COMPENSATION);
        motor.setIdleMode(DrivetrainConstants.ANGLE_NEUTRAL);

        SparkMaxPIDController PIDController = motor.getPIDController();
        PIDController.setP(AzimuthGains.kP);
        PIDController.setI(AzimuthGains.kI);
        PIDController.setD(AzimuthGains.kD);
        PIDController.setFF(AzimuthGains.kS);

        motor.burnFlash();

        return motor;
    }

    public static CANSparkMax configDriveMotor(CANSparkMax motor){
        motor.setInverted(DrivetrainConstants.DRIVE_INVERT);
        motor.setSmartCurrentLimit(DrivetrainConstants.DRIVE_SUPPLY_LIMIT);
        motor.enableVoltageCompensation(DrivetrainConstants.DRIVE_VOLTAGE_COMPENSATION);
        motor.setIdleMode(DrivetrainConstants.ANGLE_NEUTRAL);

        SparkMaxPIDController PIDController = motor.getPIDController();
        PIDController.setP(DriveGains.kP);
        PIDController.setI(DriveGains.kI);
        PIDController.setD(DriveGains.kD);
        PIDController.setFF(DriveGains.kS);

        motor.setOpenLoopRampRate(DrivetrainConstants.DRIVE_OPEN_RAMP_RATE);
        motor.setClosedLoopRampRate(DrivetrainConstants.DRIVE_CLOSED_RAMP_RATE);

        motor.burnFlash();

        return motor;
    }
}