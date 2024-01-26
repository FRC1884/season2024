package frc.robot.core.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.util.MathUtils;
import frc.robot.core.util.SimpleDeadBand;

public class GameController extends CommandXboxController {
  public static final double DEADZONE = 0.1;
  private static final MathUtils.TransformPresets function = MathUtils.TransformPresets.TRANSFORM;
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public GameController(int port) {
    super(port);
  }

  /**
   * Deadbands the left X axis on this controller with the specified {@link #DEADZONE}.
   * @return the deadbanded left X axis.
   */
  @Override
  public double getLeftX() {
    return SimpleDeadBand.signSquare(SimpleDeadBand.deadband(super.getLeftX(), DEADZONE));
  }

  /**
   * Deadbands the left Y axis on this controller with the specified {@link #DEADZONE}.
   * @return the deadbanded left Y axis.
   */
  @Override
  public double getLeftY() {
    return SimpleDeadBand.signSquare(SimpleDeadBand.deadband(super.getLeftY(), DEADZONE));
  }

  /**
   * Deadbands the right X axis on this controller with the specified {@link #DEADZONE}.
   * @return the deadbanded right X axis.
   */
  @Override
  public double getRightX() {
    return SimpleDeadBand.signSquare(SimpleDeadBand.deadband(super.getRightX(), DEADZONE));
  }

  /**
   * Deadbands the right Y axis on this controller with the specified {@link #DEADZONE}.
   * @return the deadbanded right Y axis.
   */
  @Override
  public double getRightY() {
    return SimpleDeadBand.signSquare(SimpleDeadBand.deadband(super.getRightY(), DEADZONE));
  }
}
