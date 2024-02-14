package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class ActionSetpointBuilder {
  private double rpm, angleSetpoint;

  public ActionSetpointBuilder(double rpm, double angleSetpoint) {
    this.rpm = rpm;
    this.angleSetpoint = angleSetpoint;
  }

  public ActionSetpoint build() {
    return new ActionSetpoint(rpm, angleSetpoint);
  }
}
