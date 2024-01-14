package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getTestButton();

  abstract JoystickButton getIntakeButton();

  private void registerIntake() {
  }

  @Override
  public void registerCommands() {
    registerIntake();
  }
}
