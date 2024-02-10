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
  public JoystickButton getIntakeStopButton() {
    return controller.getButton(Button.BUTTON_A);
  }
  @Override
  public JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  JoystickButton getShootButton() {
    System.out.println("hello");
    return controller.getButton(Button.BUTTON_X);
  }
  JoystickButton getShootStopButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }

  @Override
  JoystickButton getintakeReverseButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getintakeReverseButton'");
  }

}
