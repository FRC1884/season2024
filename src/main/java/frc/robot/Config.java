package frc.robot;

import frc.robot.core.util.controllers.BoardController;
import frc.robot.core.util.controllers.ButtonMap;
import frc.robot.core.util.controllers.Xbox;

public class Config {

  public static final class Subsystems {
    public static final boolean DRIVETRAIN_ENABLED = true;
    public static final boolean CLIMBER_ENABLED = false;
    public static final boolean SHOOTER_ENABLED = true;

    public static final class Intake {
      public static final boolean INTAKE_ENABLED = true;
      public static final boolean INTAKE_SENSOR_ENABLED = true;
    }

    public static final boolean FEEDER_ENABLED = true;

    public static final boolean LEDS_ENABLED = true;
  }

  public class Controllers {
    public static final boolean DRIVER_ENALBED = true;
    public static final boolean JOYSTICK_OPERATOR_ENABLED = false;
    public static final boolean OPERATOR_ENABLED = true;
    public static final boolean BOARD_OPERATOR_ENABLED = true;

    public static ButtonMap getDriverController() {
      return new Xbox();
    }

    public static ButtonMap getOperatorController() {
      return BOARD_OPERATOR_ENABLED ? new BoardController() : new Xbox();
    }
  }
}
