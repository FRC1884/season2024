package frc.robot.layout;

import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.PrinterMessageFromOperator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.subsystems.Intake;
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

  abstract JoystickButton getBoardButtonOne();
  abstract JoystickButton getBoardButtonTwo();
  abstract JoystickButton getBoardButtonThree();
  abstract JoystickButton getBoardButtonFour();
  abstract JoystickButton getBoardButtonFive();
  abstract JoystickButton getBoardButtonSix();
  abstract JoystickButton getBoardButtonSeven();
  abstract JoystickButton getBoardButtonEight();
  abstract JoystickButton getBoardButtonNine();
  abstract JoystickButton getBoardButtonTen();
  abstract JoystickButton getBoardButtonEleven();
  abstract JoystickButton getBoardButtonTwelve();
  abstract Trigger getBoardAxisX();
  abstract Trigger getBoardAxisY();
  abstract Trigger getBoardAxisXMinus();
  abstract Trigger getBoardAxisYMinus();
  abstract Trigger getBoardAxisIdle();


  private void registerPrototype() {
    if(ExampleConfig.Subsystems.PROTOTYPE_ENABLED) {
      Prototypes prototypes = Prototypes.getInstance();
      // getBoardAxisX().onTrue(prototypes.run(-0.1, 0.1, 0.7, 0.0));
      // getBoardAxisXMinus().onTrue(prototypes.run(-0.2, 0.2, 0.7, 0.0));
      // getBoardAxisY().onTrue(prototypes.run(-0.3, 0.3, 0.7, 0.0));
      // getBoardAxisYMinus().onTrue(prototypes.run(-0.4, 0.4, 0.7, 0.0));
      getBoardButtonEleven().onTrue(prototypes.run(0.0, 0.0, 0.0, 0));
      getBoardButtonOne().whileTrue(prototypes.run(0.5, 0.5, -0.5, 0.01));
      getBoardButtonTwo().whileTrue(prototypes.run(0.5, 0.5, -0.5, -0.01));
    }
  }

  private void registerIntake() {
    if(ExampleConfig.Subsystems.INTAKE_ENABLED){
    Intake intake = Intake.getInstance();
    getBoardButtonOne().onTrue(intake.run(()->0.3, ()->-0.6));
    getBoardButtonTwo().onTrue(intake.run(()->0.4, ()->-0.7));
    getBoardButtonEleven().onTrue(intake.run(()->0.0, ()->0.0));
    }
  }

  @Override
  public void registerCommands() {
    registerPrototype();
    registerIntake();
    
  }
}
