package frc.robot;

import frc.robot.core.util.controllers.GameController;
import frc.robot.layout.TwoJoyStickDriverMap;
import frc.robot.layout.TwoJoyStickOperatorMap;

public class OI {
  private static OI instance;

  public static OI getInstance() {
    if (instance == null) {
      instance = new OI();
    }
    return instance;
  }

  private GameController driver;

  public GameController getDriver() {
    return driver;
  }

  private GameController operator;

  public GameController getOperator() {
    return operator;
  }

  public void registerCommands() {
    new TwoJoyStickDriverMap(driver).registerCommands();
    new TwoJoyStickOperatorMap(operator).registerCommands();
  }

  private OI() {
    // driver = new GameController(RobotMap.ControllerMap.DRIVER_JOYSTICK, new Logitech());
    driver =
        new GameController(
            RobotMap.ControllerMap.DRIVER_JOYSTICK,
            ExampleConfig.Controllers.getDriverController());
    operator =
        new GameController(
            RobotMap.ControllerMap.OPERATOR_JOYSTICK,
            ExampleConfig.Controllers.getOperatorController());
  }
}
