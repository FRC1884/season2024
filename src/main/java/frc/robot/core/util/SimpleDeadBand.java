package frc.robot.core.util;

public class SimpleDeadBand {
    /**
   * Kills the signal if it is less than the deadband.
   *
   * @param value The value to be modified.
   * @param deadband The deadband.
   * @return The modified value.
   */
    public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
    
    /**
   * Squares the input. keeps the sign.
   *
   * @param value The value to be modified.
   * @return Deadbanded and squared value.
   */
  public static double signSquare(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
