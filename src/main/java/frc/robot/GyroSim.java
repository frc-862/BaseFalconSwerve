package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroSim {
    private double lastKnownAngle = 0.0;

    public Rotation2d getYaw() {
        return new Rotation2d(lastKnownAngle);
    }

    public void update(double omega, double dt) {
        lastKnownAngle += omega * dt;
    }
    
}
