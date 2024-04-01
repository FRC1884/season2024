package frc.robot;

import frc.robot.core.util.controllers.GameController;
import frc.robot.layout.BoardOperatorMap;
import frc.robot.layout.ChampsBoardOperator;
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
    // NamedCommands.registerCommands(
    //     new HashMap<
    //         String,
    // Command>()); // TODO no idea if this works or not. Theoretically the hashmap of strings
    // and commands takes the string input from the waypoints in the path and
    // associates that to the command that you want to run, but thats just a
    // theory, a game theory.

    new TwoJoyStickDriverMap(driver).registerCommands();
    if(Config.Controllers.BOARD_OPERATOR_ENABLED){
      new ChampsBoardOperator(operator).registerCommands();
    }
    else if (Config.Controllers.JOYSTICK_OPERATOR_ENABLED){
      new TwoJoyStickOperatorMap(operator).registerCommands();
    }
    
  }

  private OI() {
    // driver = new GameController(RobotMap.ControllerMap.DRIVER_JOYSTICK, new Logitech());
    driver =
        new GameController(
            RobotMap.ControllerMap.DRIVER_JOYSTICK,
            Config.Controllers.getDriverController());
    operator =
        new GameController(
            RobotMap.ControllerMap.OPERATOR_JOYSTICK,
            Config.Controllers.getOperatorController());
  }
}
