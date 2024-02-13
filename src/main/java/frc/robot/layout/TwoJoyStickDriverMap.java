package frc.robot.layout;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.MAXSwerve.MaxSwerveConstants;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.core.util.controllers.ButtonMap.Button;
import frc.robot.core.util.controllers.ButtonMap.Dpad;
import frc.robot.core.util.controllers.GameController;

public class TwoJoyStickDriverMap extends DriverMap {

  public TwoJoyStickDriverMap(GameController controller) {
    super(controller);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    double x =
        Math.pow(controller.getAxis(Axis.AXIS_LEFT_Y), 1)
            * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
    double y =
        Math.pow(controller.getAxis(Axis.AXIS_LEFT_X), 1)
            * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
    double rot = controller.getAxis(Axis.AXIS_RIGHT_X) * MaxSwerveConstants.kMaxAngularSpeed * 0.7;

    return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, new Rotation2d(0, 0));
  }

  @Override
  public double getSwerveXSpeed() {
    if(DriverStation.getAlliance().isPresent() 
    && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
      return -controller.getAxis(Axis.AXIS_LEFT_Y) * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
    else
      return controller.getAxis(Axis.AXIS_LEFT_Y) * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
  }

  @Override
  public double getSwerveYSpeed() {
    if(DriverStation.getAlliance().isPresent() 
      && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
      return -controller.getAxis(Axis.AXIS_LEFT_X) * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
    else
      return controller.getAxis(Axis.AXIS_LEFT_X) * MaxSwerveConstants.kMaxSpeedMetersPerSecond;
  }

  @Override
  public double getSwerveRot() {
    return -controller.getAxis(Axis.AXIS_RIGHT_X) * MaxSwerveConstants.kMaxAngularSpeed;
  }

  @Override
  public JoystickButton getArcingButton(){
    return controller.getButton(Button.BUTTON_Y); 
  }
  
  @Override
  public JoystickButton getTestButton() {
    return controller.getButton(Button.BUTTON_B);
  }

  @Override
  public JoystickButton getFollowAprilTagButton() {
    return controller.getButton(Button.BUTTON_A);
  }

  @Override
  public JoystickButton getFollowNoteButton(){
    return controller.getButton(Button.BUTTON_RIGHT_BUMPER);
  }

  // @Override
  // public JoystickButton getResetOdometryVisionButton(){
  //   return controller.getButton(Button.BUTTON_X);
  // }

  @Override
  public JoystickButton getZeroGyroButton(){
    return controller.getButton(Button.BUTTON_X);
  }

  @Override
  public JoystickButton getSpeakerBlueButton(){
    return controller.getButton(Button.BUTTON_LEFT_BUMPER);
  }
 @Override
  public JoystickButton getAmpBlueButton(){
    return controller.getButton(Button.BUTTON_OPTIONS);
  }

  @Override
  public JoystickButton getNavigateAndAllignButton(){
    return controller.getButton(Button.BUTTON_SHARE);
  }
    
  @Override
  public void registerCommands() {
    super.registerCommands();
  }
}
