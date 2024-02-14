package frc.robot;

import frc.robot.core.util.controllers.BoardController;
import frc.robot.core.util.controllers.ButtonMap;
import frc.robot.core.util.controllers.Xbox;
import frc.robot.layout.BoardOperatorMap;

public class ExampleConfig {
  public class Subsystems {
    public static final boolean DRIVETRAIN_ENABLED = true;
    public static final boolean CLIMBER_ENABLED = false;
    public static final boolean SHAMPER_ENABLED = true;
    public static final boolean INTAKE_ENABLED = true;
    public static final boolean PROTOTYPE_ENABLED = false;
    public static final boolean LEDS_ENABLED = false;
  }

  public class Controllers {
    public static final boolean DRIVER_ENALBED = true;
    public static final boolean JOYSTICK_OPERATOR_ENABLED = true;
    public static final boolean OPERATOR_ENABLED = true;
    public static final boolean BOARD_OPERATOR_ENABLED = !JOYSTICK_OPERATOR_ENABLED;

    public static ButtonMap getDriverController() {
      return new Xbox();
    }

    public static ButtonMap getOperatorController() {
      return BOARD_OPERATOR_ENABLED ? new BoardController() : new Xbox();
    }
  }
}
