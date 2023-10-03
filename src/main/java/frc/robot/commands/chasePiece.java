// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.auto.AutonomousCommandFactory;
import frc.lib.pathplanner.com.pathplanner.lib.PathPoint;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class chasePiece extends CommandBase {

	LimelightSubsystem limelight;
	AutonomousCommandFactory autoFactory;
	Swerve drivetrain;

	public chasePiece(LimelightSubsystem limelight, AutonomousCommandFactory autoFactory, Swerve drivetrain) {
		this.limelight = limelight;
		this.autoFactory = autoFactory;
		this.drivetrain = drivetrain;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		autoFactory.createManualTrajectory(AutonomousConstants.CUBE_CHASE_CONSTRAINTS,new PathPoint(drivetrain.getPose().getTranslation(), drivetrain.getYaw()), new PathPoint(new Translation2d(0, 0), new Rotation2d(0,0)));
	}
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
