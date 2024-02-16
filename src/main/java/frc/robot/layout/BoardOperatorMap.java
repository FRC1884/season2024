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
  JoystickButton getIntakeButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getOuttakeButton() {
    return controller.getButton(Button.BUTTON_START);
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
  JoystickButton getPivotButtonOne() {
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  @Override
  JoystickButton getPivotButtonTwo() {
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }

  @Override
  JoystickButton getFeederButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  double getClimberAxis() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }

  @Override
  double getManualPivotAxis() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  JoystickButton getLEDPatternOneButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  JoystickButton getLEDPatternTwoButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  JoystickButton getLEDPatternOffButton() {
    return controller.getButton(Button.BUTTON_EXTRA_2);
  }

  @Override
  JoystickButton getLEDPatternThreeButton() {
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  JoystickButton getLEDPatternFourButton() {
    return controller.getButton(Button.BUTTON_Y);
  }

  @Override
  JoystickButton getLEDPatternFiveButton() {
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
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