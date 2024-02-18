package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class ActionSetpointBuilder {
  private double vK, angleSetpoint, vF;

  public ActionSetpointBuilder(double vK, double angleSetpoint, double vF) {
    this.vK = vK;
    this.angleSetpoint = angleSetpoint;
    this.vF = vF;
  }

  public ActionSetpoint build() {
    return new ActionSetpoint(vK, angleSetpoint, vF);
  }
}
