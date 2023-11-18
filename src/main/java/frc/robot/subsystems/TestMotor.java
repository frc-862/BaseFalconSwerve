// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.REVConfigs;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TestMotor extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(6, MotorType.kBrushless);

  public TestMotor() {
    // motor = REVConfigs.configDriveMotor(motor);
    motor = REVConfigs.configAngleMotor(motor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double pow){
    motor.set(pow);
  }

  public void stop(){
    setPower(0);
  }
}
