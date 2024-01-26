package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.GameController;

public class TwoJoyStickOperatorMap extends OperatorMap {
  public TwoJoyStickOperatorMap(GameController controller) {
    super(controller);
  }

  @Override
  public Trigger getTestButton() {
    return controller.a();
  }

  @Override
  public Trigger getIntakeButton() {
    return controller.b();
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
