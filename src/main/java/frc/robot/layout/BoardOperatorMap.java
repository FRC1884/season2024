package frc.robot.layout;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
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
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_B);
  }
  @Override
  JoystickButton getintakeReverseButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  JoystickButton getShootButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getShootStopButton() {
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }

  @Override
  JoystickButton getPivotButtonOne() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  JoystickButton getPivotButtonTwo(){
    return controller.getButton(Button.BUTTON_SHARE);
  }

  @Override
  JoystickButton getFeederButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  JoystickButton getFeederStopButton() {
    return controller.getButton(Button.BUTTON_START);
  }

  @Override
  double getClimberAxis() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }


}
