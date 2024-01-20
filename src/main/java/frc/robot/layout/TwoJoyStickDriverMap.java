package frc.robot.layout;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.MAXSwerve.MaxSwerveConstants;
import frc.robot.core.TalonSwerve.SwerveConstants;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public double getKitDrivetrainRot() {
    return controller.getAxis(Axis.AXIS_RIGHT_X);
  }

  @Override
  public double getKitDrivetrainForward() {
    return controller.getAxis(Axis.AXIS_LEFT_Y);
  }

  @Override
  public void registerCommands() {
    super.registerCommands();
  }

}