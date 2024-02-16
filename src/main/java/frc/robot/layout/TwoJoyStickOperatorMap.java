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
  public JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getShootSpeakerButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getShootAmpButton() {
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  JoystickButton getOuttakeButton() {
    return controller.getButton(Button.BUTTON_SHARE);
  }

  @Override
  JoystickButton getPivotButtonOne() {
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }

  @Override
  JoystickButton getPivotButtonTwo() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  JoystickButton getFeederButton() {
    return controller.getButton(Button.BUTTON_X);
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
  JoystickButton getLEDPatternThreeButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getLEDPatternThreeButton'");
  }

  @Override
  JoystickButton getLEDPatternFourButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getLEDPatternFourButton'");
  }

  @Override
  JoystickButton getLEDPatternFiveButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getLEDPatternFiveButton'");
  }

  
  @Override
  double getLEDAxis1() {
    return controller.getAxis(Axis.AXIS_LEFT_X);
  }

  
  @Override
  double getLEDAxis2() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }
}
