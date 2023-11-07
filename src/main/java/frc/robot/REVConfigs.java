package frc.robot;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.AzimuthGains;
import frc.robot.Constants.DrivetrainConstants.DriveGains;

public class REVConfigs {
    public CANSparkMax configAngleMotor(CANSparkMax motor){
        motor.setInverted(DrivetrainConstants.ANGLE_INVERT);
        motor.setSmartCurrentLimit(DrivetrainConstants.AZIMUTH_SUPPLY_LIMIT);
        motor.enableVoltageCompensation(DrivetrainConstants.AZIMUTH_VOLTAGE_COMPENSATION);
        motor.setIdleMode(DrivetrainConstants.ANGLE_NEUTRAL);

        motor.getPIDController().setP(AzimuthGains.kP);
        motor.getPIDController().setI(AzimuthGains.kI);
        motor.getPIDController().setD(AzimuthGains.kD);
        motor.getPIDController().setFF(AzimuthGains.kS);

        motor.burnFlash();

        return motor;
    }

public CANSparkMax configDriveMotor(CANSparkMax motor){
        motor.setInverted(DrivetrainConstants.DRIVE_INVERT);
        motor.setSmartCurrentLimit(DrivetrainConstants.DRIVE_SUPPLY_LIMIT);
        motor.enableVoltageCompensation(DrivetrainConstants.DRIVE_VOLTAGE_COMPENSATION);
        motor.setIdleMode(DrivetrainConstants.ANGLE_NEUTRAL);

        motor.getPIDController().setP(DriveGains.kP);
        motor.getPIDController().setI(DriveGains.kI);
        motor.getPIDController().setD(DriveGains.kD);
        motor.getPIDController().setFF(DriveGains.kS);

        motor.setOpenLoopRampRate(DrivetrainConstants.DRIVE_OPEN_RAMP_RATE);
        motor.setClosedLoopRampRate(DrivetrainConstants.DRIVE_CLOSED_RAMP_RATE);

        motor.burnFlash();

        return motor;
    }
}