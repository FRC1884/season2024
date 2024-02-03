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

  // @Override
  // JoystickButton getBoardButtonOne() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonOne'");
  // }

  // @Override
  // JoystickButton getBoardButtonTwo() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonTwo'");
  // }

  // @Override
  // JoystickButton getBoardButtonThree() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonThree'");
  // }

  // @Override
  // JoystickButton getBoardButtonFour() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonFour'");
  // }

  // @Override
  // JoystickButton getBoardButtonFive() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonFive'");
  // }

  // @Override
  // JoystickButton getBoardButtonSix() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonSix'");
  // }

  // @Override
  // JoystickButton getBoardButtonSeven() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonSeven'");
  // }

  // @Override
  // JoystickButton getBoardButtonEight() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonEight'");
  // }

  // @Override
  // JoystickButton getBoardButtonNine() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonNine'");
  // }

  // @Override
  // JoystickButton getBoardButtonTen() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonTen'");
  // }

  // @Override
  // JoystickButton getBoardButtonEleven() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonEleven'");
  // }

  // @Override
  // JoystickButton getBoardButtonTwelve() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardButtonTwelve'");
  // }

  // @Override
  // Trigger getBoardAxisX() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardAxisX'");
  // }

  // @Override
  // Trigger getBoardAxisY() {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'getBoardAxisY'");
  // }
}