package frc.robot.layout;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.ShamperMap;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Vision.Vision;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }


  abstract double getSwerveXSpeed();

  abstract double getSwerveYSpeed();

  abstract double getSwerveRot();

  abstract JoystickButton getSlowModeToggleButton();
  
  abstract JoystickButton getArcingButton();

  abstract JoystickButton getRaiseShampreButton();

  abstract JoystickButton getFollowAprilTagButton();

  abstract JoystickButton getFollowNoteButton();

  abstract JoystickButton getZeroGyroButton();

  abstract JoystickButton getNavigateAndAllignAmpButton();

  abstract JoystickButton getNavigateAndAllignStageButton();

  private void registerDrivetrain() {
    if (Config.Subsystems.DRIVETRAIN_ENABLED) {
      var drivetrain = Drivetrain.getInstance();
      var vision = Vision.getInstance();

      //--- Drive --- 
      drivetrain.setDefaultCommand(
      drivetrain.driveCommand(
              this::getSwerveXSpeed, this::getSwerveYSpeed, this::getSwerveRot));
      
      //--- Alignment ---
      
        getArcingButton().whileTrue(drivetrain.alignWhileDrivingCommand(
              this::getSwerveXSpeed,this::getSwerveYSpeed, () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Coordinates.RED_SPEAKER.getTranslation().plus(ShamperMap.SHOT_OFFSET) : Coordinates.BLUE_SPEAKER.getTranslation().minus(ShamperMap.SHOT_OFFSET)));
      
      
      getNavigateAndAllignAmpButton().whileTrue(drivetrain.pathFindThenFollowPathCommand(
        "Go To Amp"));

      getNavigateAndAllignAmpButton().whileTrue(drivetrain.pathFindThenFollowPathCommand("Go To Stage"));
        
      getFollowNoteButton().whileTrue(vision.PIDtoNoteRobotRelativeCommand());
      getZeroGyroButton().onTrue(drivetrain.zeroYawCommand());
    }
  }

  private void shamper(){
      Pivot pivot = Pivot.getInstance();
      getRaiseShampreButton().onTrue(pivot.updatePosition(() -> 70.0));
  }


  @Override
  public void registerCommands() {
    registerDrivetrain();
    shamper();
  }
}