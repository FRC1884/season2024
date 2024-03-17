package frc.robot.layout;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.RobotMap.Coordinates;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.Vision;

import java.util.function.Supplier;

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
            drivetrain.setDefaultCommand(drivetrain.driveCommand(this::getSwerveXSpeed, this::getSwerveYSpeed, this::getSwerveRot));

            //--- Arcing ---

            // defaults to red speaker to avoid null pointer exception
            Supplier<Translation2d> getTarget = () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                    ? Coordinates.BLUE_SPEAKER.getTranslation()
                    : Coordinates.RED_SPEAKER.getTranslation();

            getArcingButton().whileTrue(drivetrain.alignWhileDrivingCommand(this::getSwerveXSpeed, this::getSwerveYSpeed, getTarget));

            // getNavigateAndAllignAmpButton().whileTrue(drivetrain.pathFindThenFollowPathCommand(
            //   "Go To Amp"));

            // getNavigateAndAllignAmpButton().whileTrue(drivetrain.pathFindThenFollowPathCommand("Go To Stage"));

            getFollowNoteButton().whileTrue(vision.PIDtoNoteRobotRelativeCommand());
            getZeroGyroButton().onTrue(drivetrain.zeroYawCommand());
        }
    }

    @Override
    public void registerCommands() {
        registerDrivetrain();
    }
}
