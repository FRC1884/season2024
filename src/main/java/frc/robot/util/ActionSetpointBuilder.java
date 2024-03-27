package frc.robot.util;

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
