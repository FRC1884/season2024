package frc.robot.layout;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.RobotMap;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.PrototypeMap;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Prototypes;
import frc.robot.subsystems.test;
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

  abstract JoystickButton getTestButton();

  abstract JoystickButton getFollowAprilTagButton();

  abstract JoystickButton getFollowNoteButton();

  abstract JoystickButton getZeroGyroButton();

  abstract JoystickButton getNavigateAndAllignAmpButton();

  abstract JoystickButton getNavigateAndAllignStageButton();

  private void registerDrivetrain() {
    if (Config.Subsystems.DRIVETRAIN_ENABLED) {
      System.out.println("Register Drivetrain");
      var drivetrain = Drivetrain.getInstance();
      var vision = Vision.getInstance();

      //--- Drive --- 
      drivetrain.setDefaultCommand(
      drivetrain.driveCommand(
      this::getSwerveXSpeed, this::getSwerveYSpeed, this::getSwerveRot));

      //--- Arcing ---
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        getArcingButton().whileTrue(drivetrain.alignWhileDrivingCommand(
              this::getSwerveXSpeed,this::getSwerveYSpeed, () -> Coordinates.RED_SPEAKER.getTranslation()));
      }
      else{
        getArcingButton().whileTrue(drivetrain.alignWhileDrivingCommand(
              this::getSwerveXSpeed,this::getSwerveYSpeed, () -> Coordinates.BLUE_SPEAKER.getTranslation()));
      }
      
      getNavigateAndAllignAmpButton().whileTrue(drivetrain.navigateAndAlignCommand(
        "Go To Amp", () -> Coordinates.RED_AMP.getTranslation()));

      getNavigateAndAllignAmpButton().whileTrue(drivetrain.navigateAndAlignCommand(
        "Go To Stage", () -> Coordinates.RED_STAGE.getTranslation()));
        
      getFollowNoteButton().whileTrue(vision.followNoteCommand());
      getZeroGyroButton().onTrue(drivetrain.zeroYawCommand());
    }
  }

  @Override
  public void registerCommands() {
    registerDrivetrain();
  }
}
