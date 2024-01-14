package frc.robot.util;


import edu.wpi.first.math.geometry.Pose2d;


public class ActionSetpoint {
    private double angle;
    private Pose2d pose;

    ActionSetpoint(Pose2d pose, double angle) {
        this.pose = pose;
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }

    public Pose2d getPose() {
        return pose;
    }
}



