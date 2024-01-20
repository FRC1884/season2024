package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.GameController;

public class TwoJoyStickOperatorMap extends OperatorMap {
  public TwoJoyStickOperatorMap(GameController controller) {
    super(controller);
  }

  @Override
  public JoystickButton getTestButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  public JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_B);
  }
 @Override
  public JoystickButton getFlywheelButton(){
    return controller.getButton(Button.BUTTON_Y);

  }

  @Override
  public void registerCommands() {

    super.registerCommands();
  }

}
