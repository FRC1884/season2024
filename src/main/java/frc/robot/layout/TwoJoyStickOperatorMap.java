package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.ButtonMap.Dpad;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Prototypes;

public class TwoJoyStickOperatorMap extends OperatorMap {
  public TwoJoyStickOperatorMap(GameController controller) {
    super(controller);
  }
  @Override
  public void registerCommands() {
    super.registerCommands();
  }

  @Override
  public JoystickButton getIntakeStopButton() {
    return controller.getButton(Button.BUTTON_X);
  }
  @Override
  public JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getShootButton() {
    return controller.getButton(Button.BUTTON_START);
  }
  JoystickButton getShootStopButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  JoystickButton getShootAmpButton() {
    return controller.getButton(Button.BUTTON_B);
  }



  @Override
  JoystickButton getintakeReverseButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  JoystickButton getPivotButtonOne() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  JoystickButton getPivotButtonTwo(){
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getFeederButton() {
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }

  @Override
  JoystickButton getFeederStopButton() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  double getClimberAxis() {
    return controller.getAxis(Axis.AXIS_RIGHT_Y);
  }

  @Override
  double getManualPivotAxis() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }

  @Override
  JoystickButton getLEDPatternOneButton() {
    return null;
  }

  @Override
  JoystickButton getLEDPatternTwoButton() {
    return null;
  }

  @Override
  JoystickButton getLEDPatternOffButton() {
    return null;
  }
  @Override
  JoystickButton getPivotButtonOff() {
    return controller.getButton(Button.BUTTON_A);
  }

}
