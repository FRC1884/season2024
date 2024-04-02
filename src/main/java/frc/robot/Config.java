package frc.robot;

import frc.robot.core.util.controllers.BoardController;
import frc.robot.core.util.controllers.ButtonMap;
import frc.robot.core.util.controllers.Xbox;
import frc.robot.subsystems.pivot.PivotHardware;

public class Config {

    enum RobotType {
        DEV, COMP
    }

    public static final RobotType ROBOT_TYPE = RobotType.COMP;

    public static final class Subsystems {
        public static final boolean DRIVETRAIN_ENABLED = true;
        public static final boolean CLIMBER_ENABLED = false;
        public static final boolean SHOOTER_ENABLED = true;
        // TODO: check if this actually works
        public static final boolean VISION_ENABLED = false;
        public static final boolean PIVOT_ENABLED = true;

        public static final PivotHardware.PivotHardwareType PIVOT_HARDWARE_TYPE = PivotHardware.PivotHardwareType.DUAL_ACTUATOR;

        public static final boolean INTAKE_ENABLED = true;
        public static final boolean FEEDER_ENABLED = true;

        public static final boolean LEDS_ENABLED = true;
    }

    public class Controllers {
        public static final boolean DRIVER_ENALBED = true;
        public static final boolean JOYSTICK_OPERATOR_ENABLED = false;
        public static final boolean OPERATOR_ENABLED = false;
        public static final boolean BOARD_OPERATOR_ENABLED = true;

        public static ButtonMap getDriverController() {
            return new Xbox();
        }

        public static ButtonMap getOperatorController() {
            return BOARD_OPERATOR_ENABLED ? new BoardController() : new Xbox();
        }
    }

    public static final class Auto {
        public static final boolean IS_AUTO_OVERRIDE_SPEAKER = true;
    }
}
