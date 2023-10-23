package frc.robot.subsystems;

import java.util.concurrent.ExecutionException;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.vision.Limelight;
import frc.thunder.vision.Limelight.CamMode;
import frc.thunder.vision.Limelight.LEDMode;
import frc.thunder.vision.Limelight.StreamMode;

public class LimelightSubsystem extends SubsystemBase {
    private Limelight limelight;
    private int count = 0;
    public LimelightSubsystem() {
        this.limelight = new Limelight("limelight");
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        if (((int) Timer.getFPGATimestamp()) % 10 == 0) {
            // System.out.println("HasTarget: " + limelight.hasTarget());
            // System.out.println("getTargetY: " + limelight.getTargetY());
            // System.out.println("getTargetX: " + limelight.getTargetX());
            // System.out.println("getTargetArea: " + limelight.getTargetArea());
            // System.out.println("getPipelineLatency: " + limelight.getPipelineLatency());
            // System.out.println("getCaptureLatency: " + limelight.getCaptureLatency());
            // System.out.println("getTotalLatency: " + limelight.getTotalLatency());
            // System.out.println("getTShort: " + limelight.getTShort());
            // System.out.println("getTLong: " + limelight.getTLong());
            // System.out.println("getTHor: " + limelight.getTHor());
            // System.out.println("getTVert: " + limelight.getTVert());
            // System.out.println("getPipeline: " + limelight.getPipeline());
            // System.out.println("getTargetJson: " + limelight.getTargetJSON());
            // System.out.println("getClassID: " + limelight.getNeuralClassID());
            // System.out.println("getAverageHSV: " + limelight.getAverageHSV().toString());
            // System.out.println("getBotPose: " + limelight.getAlliancePose());
            // System.out.println("getCamPoseTargetSpace: " + limelight.getCamPoseTargetSpace());
            // System.out.println("getCamPoseRobotSpace: " + limelight.getCamPoseRobotSpace());
            // System.out.println("getTargetPoseCameraSpace: " + limelight.getTargetPoseCameraSpace());
            // System.out.println("getTargetPoseRobotSpace: " + limelight.getTargetPoseRobotSpace());
            // System.out.println("getApriltagID: " + limelight.getApriltagID());
            // //Need to test ledmode setting, cammode, streamode, pipeline, crop, 
            // System.out.println("getBaseURL: " + limelight.getBaseUrl().toString());

            // System.out.println("snapshots: " + limelight.getSnapshotNames());
            // // System.out.println("hwreport: " + limelight.getHWReport());

            //     limelight.deleteAllSnapshots();
            // count += 1;
        }
    }
}
