package frc.robot.core.util;

import static frc.robot.core.util.controllers.GameController.DEADZONE;
import static java.lang.Math.*;

/**
 * A util class containing a multitude of ramp and deadzone functions, primarily for use with
 * controllers. <br>
 * <br>
 * Contains quite a few customizable options here to cater to the drivers' preferences for joystick
 * control.
 */
public class MathUtils {
  @SuppressWarnings("unused")
  public enum TransformPresets {
    LINEAR(
        (value, deadband) ->
            (abs(value) > deadband)
                ? (value + deadband * Math.signum(value)) / (1.0 - deadband)
                : 0.0),
    TRANSFORM((value, jTF) -> (1.0 / (jTF - 1)) * signum(value) * (Math.pow(jTF, abs(value)) - 1)),
    SQUARE((value, none) -> Math.copySign(Math.pow(deadBand.on(value, DEADZONE), 2), value)),
    CUBE(
        (value, weight) ->
            (min(abs(weight), 1.0) * Math.pow(value, 3)) + ((1 - min(abs(weight), 1.0)) * value));

    private final TransformFunction func;

    TransformPresets(TransformFunction func) {
      this.func = func;
    }

    public double on(double value, double deadzone) {
      return func.deadBand(value, deadzone);
    }
  }

  public interface TransformFunction {
    double on(double val, double weight);

    default double deadBand(double val, double deadzone) {
      val = deadBand.on(val, DEADZONE);
      double f = signum(val) * this.on(abs(val) - DEADZONE, deadzone);
      double d = this.on(1 - DEADZONE, deadzone);

      val = f / d;

      return this.on(val, deadzone);
    }
  }

  /**
   * Kills the signal if it is less than the deadband. This is auto-integrated into every other
   * function.
   *
   * @param value The value to be modified.
   * @param deadband The deadband.
   * @return The modified value.
   */
  public static TransformFunction deadBand =
      (value, deadband) ->
          (abs(value) > deadband)
              ? (value + deadband * Math.signum(value)) / (1.0 - deadband)
              : 0.0;
}
