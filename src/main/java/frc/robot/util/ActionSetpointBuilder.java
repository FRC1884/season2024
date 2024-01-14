package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class ActionSetpointBuilder {
    private Pose2d pose;
    private double angle;

    public ActionSetpointBuilder(double x, double y, double rot, double angle) {
        this.pose = new Pose2d(x, y, new Rotation2d(rot));
        this.angle = angle;
    }

    public ActionSetpointBuilder(SetpointMap.FieldPoses position, double angle) {
        this.pose = position.getPose();
        this.angle = angle;
    }

    public ActionSetpointBuilder onAlliance(DriverStation.Alliance alliance) {
        return this;
    }

    public ActionSetpoint build() {
        return new ActionSetpoint(pose, angle);
    }
}
