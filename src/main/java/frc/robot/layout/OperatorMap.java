package frc.robot.layout;

import javax.print.attribute.standard.PrinterMessageFromOperator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.subsystems.Prototypes;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.BoardController;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getResetButton();

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getShootButton();

  // abstract JoystickButton getBoardButtonOne();
  // abstract JoystickButton getBoardButtonTwo();
  // abstract JoystickButton getBoardButtonThree();
  // abstract JoystickButton getBoardButtonFour();
  // abstract JoystickButton getBoardButtonFive();
  // abstract JoystickButton getBoardButtonSix();
  // abstract JoystickButton getBoardButtonSeven();
  // abstract JoystickButton getBoardButtonEight();
  // abstract JoystickButton getBoardButtonNine();
  // abstract JoystickButton getBoardButtonTen();
  // abstract JoystickButton getBoardButtonEleven();
  // abstract JoystickButton getBoardButtonTwelve();
  // abstract Trigger getBoardAxisX();
  // abstract Trigger getBoardAxisY();



  private void registerPrototype() {
    if(ExampleConfig.Subsystems.PROTOTYPE_ENABLED) {
      Prototypes prototypes = Prototypes.getInstance();
      getResetButton().onTrue(prototypes.run(0.0, 0.0, 0.0, 0.0));
      getIntakeButton().onTrue(prototypes.run(0.0, 0.0, 1.0, 1.0));
      getShootButton().onTrue(prototypes.run(1.0, 1.0, 0.0, 0.0));
    }
  }

  @Override
  public void registerCommands() {
    registerPrototype();
    // getBoardAxisX().onTrue(new PrintCommand("x-axis works"));
    // getBoardAxisY().onTrue(new PrintCommand("y-axis works"));
    // getBoardButtonOne().whileTrue(new PrintCommand("Button 1 works"));
    // getBoardButtonTwo().whileTrue(new PrintCommand("Button 2 works"));
    // getBoardButtonThree().whileTrue(new PrintCommand("Button 3 works"));
    // getBoardButtonFour().whileTrue(new PrintCommand("Button 4 works"));
    // getBoardButtonFive().whileTrue(new PrintCommand("Button 5 works"));
    // getBoardButtonSix().whileTrue(new PrintCommand("Button 6 works"));
    // getBoardButtonSeven().whileTrue(new PrintCommand("Button 7 works"));
    // getBoardButtonEight().whileTrue(new PrintCommand("Button 8 works"));
    // getBoardButtonNine().whileTrue(new PrintCommand("Button 9 works"));
    // getBoardButtonTen().whileTrue(new PrintCommand("Button 10 works"));
    // getBoardButtonEleven().whileTrue(new PrintCommand("Button 11 works"));
    // getBoardButtonTwelve().whileTrue(new PrintCommand("Button 12 works"));

    
  }
}
