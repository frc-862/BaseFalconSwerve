// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.REVConfigs;
import frc.thunder.config.FalconConfig;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TestMotor extends SubsystemBase {
  // private CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushless);
  private TalonFX motor1;
  private TalonFX motor2;

  public TestMotor() {
    // motor = REVConfigs.configDriveMotor(motor);
    // motor = REVConfigs.configAngleMotor(motor);
    motor1 = FalconConfig.createMotor(10, false, 0, 0, NeutralModeValue.Coast);
    motor2 = FalconConfig.createMotor(11, false, 0, 0, NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double pow){
    motor1.set(pow);
    motor2.set(pow);
  }

  public void stop(){
    setPower(0);
  }
}
