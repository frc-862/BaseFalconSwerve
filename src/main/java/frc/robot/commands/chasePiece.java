// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.auto.AutonomousCommandFactory;
import frc.lib.pathplanner.com.pathplanner.lib.PathPoint;
import frc.lib.util.Limelight;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class chasePiece extends CommandBase {

	Limelight limelight;
	AutonomousCommandFactory autoFactory;
	Swerve drivetrain;

	public chasePiece(Limelight limelight, AutonomousCommandFactory autoFactory, Swerve drivetrain) {
		this.limelight = limelight;
		this.autoFactory = autoFactory;
		this.drivetrain = drivetrain;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		autoFactory.createManualTrajectory(AutonomousConstants.CUBE_CHASE_CONSTRAINTS,
		new PathPoint(drivetrain.getPose().getTranslation(), drivetrain.getYaw()),
		 new PathPoint(drivetrain.getPose().relativeTo(limelight.getTargetPoseRobotSpace().toPose2d()).getTranslation(), new Rotation2d(180,0)));
		 // TODO: Get the Translation2d from the limelight compared to the drivetrain pose
		 // Rotation2d should be 180 degrees since the starting position is facing the opposite direction, but not sure
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
