package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.KonsMotors;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getTestButton();

  abstract JoystickButton getIntakeButton();

  @Override
  public void registerCommands() {
    if (ExampleConfig.Subsystems.INTAKE_ENABLED) {
      KonsMotors newMotors = KonsMotors.getInstance();
      getIntakeButton().onTrue(!newMotors.isRunning() ? newMotors.run() : newMotors.kill());
    }
  }
}
