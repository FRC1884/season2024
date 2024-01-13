package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.core.util.controllers.GameController;
import frc.robot.layout.TwoJoyStickDriverMap;
import frc.robot.layout.TwoJoyStickOperatorMap;
import java.util.HashMap;

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
    NamedCommands.registerCommands(
        new HashMap<
            String,
            Command>()); // TODO no idea if this works or not. Theoretically the hashmap of strings
    // and commands takes the string input from the waypoints in the path and
    // associates that to the command that you want to run, but thats just a
    // theory, a game theory.
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
