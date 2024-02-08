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
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getResetButton'");
  }

  @Override
  JoystickButton getIntakeButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getIntakeButton'");
  }

  @Override
  JoystickButton getShootButton() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getShootButton'");
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }

}
