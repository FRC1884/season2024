package frc.robot.util;

public class ActionSetpoint {
  private double rpm, angleSetpoint;

  ActionSetpoint(double rpm, double aS) {
    this.rpm = rpm;
    this.angleSetpoint = aS;
  }

  public double getRPM() {
    return rpm;
  }

  public double getAngleSetpoint() {
    return angleSetpoint;
  }
}
