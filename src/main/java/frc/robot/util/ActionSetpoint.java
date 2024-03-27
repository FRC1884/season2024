package frc.robot.util;

public class ActionSetpoint {

  public static final ActionSetpoint SUBWOOFER_SHOT = new ActionSetpoint(2600, -125);
  public static final ActionSetpoint PODIUM_SHOT = new ActionSetpoint(4000, -90); // TODO - Needs to be changed

  private final double rpm, angleSetpoint;

  ActionSetpoint(double rpm, double aS) {
    this.rpm = rpm;
    this.angleSetpoint = aS;
  }

  public double getRPM() {
    return rpm;
  }

  public double getAngle() {
    return angleSetpoint;
  }
}
