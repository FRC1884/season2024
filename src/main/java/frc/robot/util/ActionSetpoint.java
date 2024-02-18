package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ActionSetpoint {
  private double vK, angleSetpoint, vF;

  ActionSetpoint(double vK, double aS, double vF) {
    this.vK = vK;
    this.angleSetpoint = aS;
    this.vF = vF;
  }

  public double getFlywheelV() {
    return vK;
  }

  public double getFeederV() {
    return vF;
  }

  public double getAngleSetpoint() {
    return angleSetpoint;
  }
}
