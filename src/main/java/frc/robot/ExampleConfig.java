package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.core.util.controllers.ButtonMap;
import frc.robot.core.util.controllers.Xbox;

public class ExampleConfig {
  public class Subsystems {
    public static final boolean DRIVETRAIN_ENABLED = true;
    public static final boolean ELEVATOR_ENABLED = false;
    public static final boolean FLYWHEEL_ENABLED = false;
    public static final boolean INTAKE_ENABLED = false;
  }

  public class Controllers {
    public static final boolean DRIVER_ENALBED = true;
    public static final boolean OPERATOR_ENABLED = false;
  }
}

