package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.BoardController;
import frc.robot.core.util.controllers.GameController;

public class BoardOperatorMap extends OperatorMap {
    public BoardOperatorMap(GameController controller) {
    super(controller);
  }

  @Override
  JoystickButton getResetButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  JoystickButton getShootButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

}
