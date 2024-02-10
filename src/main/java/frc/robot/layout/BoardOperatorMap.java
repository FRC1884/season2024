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
  JoystickButton getIntakeStopButton() {
    return controller.getButton(Button.BUTTON_EXTRA_2);
  }

  @Override
  JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getShootButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  JoystickButton getShootStopButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getintakeReverseButton() {
    return controller.getButton(Button.BUTTON_B);
  }

}
