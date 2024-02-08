package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Prototypes;

public class TwoJoyStickOperatorMap extends OperatorMap {
  public TwoJoyStickOperatorMap(GameController controller) {
    super(controller);
  }

  @Override
  public JoystickButton getResetButton() {
    return controller.getButton(Button.BUTTON_A);
  }
  @Override
  public JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }

  @Override
  JoystickButton getShootButton() {
    System.out.println("hello");
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  JoystickButton getBoardButtonOne() {
    return controller.getButton(Button.BUTTON_EXTRA_2);
  }

  @Override
  JoystickButton getBoardButtonTwo() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getBoardButtonThree() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getBoardButtonFour() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getBoardButtonFive() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getBoardButtonSix() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getBoardButtonSeven() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getBoardButtonEight() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getBoardButtonNine() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getBoardButtonTen() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getBoardButtonEleven() {
    return controller.getButton(Button.BUTTON_TOUCHPAD);
  }

  @Override
  JoystickButton getBoardButtonTwelve() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  Trigger getBoardAxisX() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  Trigger getBoardAxisY() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  Trigger getBoardAxisXMinus() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  Trigger getBoardAxisYMinus() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  Trigger getBoardAxisIdle() {
    return controller.getButton(Button.BUTTON_A);
  }
}