package frc.robot.layout;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config;
import frc.robot.Commands.IntakeUntilLoadedCommand;
import frc.robot.Commands.OptionalVisionIntakeCommand;
import frc.robot.Commands.VisionIntakeThenReturnCommand;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.DriveMap;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.AddressableLEDLights;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.pivot.Pivot;

import java.util.function.Supplier;

import javax.swing.DefaultFocusManager;

import com.fasterxml.jackson.databind.ObjectMapper.DefaultTypeResolverBuilder;

public abstract class DriverMap extends CommandMap {

    public DriverMap(GameController controller) {
        super(controller);
    }


    abstract double getSwerveXSpeed();

    abstract double getSwerveYSpeed();

    abstract double getSwerveRot();

    abstract Trigger getSlowModeToggleButton();

    abstract Trigger getArcingButton();

    abstract Trigger getFerryArcButton();

    abstract Trigger getTestButton();

    abstract Trigger getFollowAprilTagButton();

    abstract Trigger getFollowNoteButton();

    abstract Trigger getZeroGyroButton();

    abstract Trigger getNavigateAndAllignAmpButton();

    abstract Trigger getNavigateAndAllignStageButton();

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
            //getArcingButton().whileTrue(drivetrain.alignWhileDrivingCommand(this::getSwerveXSpeed, this::getSwerveYSpeed, getTarget, ));
            getFerryArcButton().whileTrue(drivetrain.alignWhileDrivingCommand(this::getSwerveXSpeed, this::getSwerveYSpeed, () -> getTarget.get(), () -> Rotation2d.fromDegrees(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue? -30 : -15)));

            // getNavigateAndAllignAmpButton().whileTrue(drivetrain.pathFindThenFollowPathCommand(
            //   "Go To Amp"));

            // getNavigateAndAllignAmpButton().whileTrue(drivetrain.pathFindThenFollowPathCommand("Go To Stage"));
            // TODO DELETE ME!
            getSlowModeToggleButton().toggleOnTrue(Commands.runOnce(() -> DriveMap.IS_SLOWMODE_ENABLED = !DriveMap.IS_SLOWMODE_ENABLED));
            getFollowNoteButton().whileTrue(vision.PIDtoNoteRobotRelativeCommand());


            getZeroGyroButton().onTrue(drivetrain.zeroYawCommand());
            getNavigateAndAllignAmpButton().whileTrue(drivetrain.pathFindThenFollowPathCommand("Amp Align"));
            
            SequentialCommandGroup visionIntakeCommand = new VisionIntakeThenReturnCommand();
            
            //getTestButton().onTrue(new DeferredCommand(() -> visionIntakeCommand, visionIntakeCommand.getRequirements()));
            // getTestButton().whileTrue(drivetrain.chasePoseRobotRelativeAndReturnCommand(vision::getRobotRelativeNotePose2d,
            //     Feeder.getInstance()::isNoteLoaded
            // ).alongWith(new IntakeUntilLoadedCommand()));
            // getTestButton().whileTrue(drivetrain.chasePoseRobotRelativeCommand_Y_WithXSupplier(vision::getRobotRelativeNotePose2d, 
            // () -> drivetrain::getChassisSpeeds.vxMetersPerSecond, Feeder.getInstance()::isNoteLoaded));


        }
    }

    private void registerLEDs() {
        var lights = AddressableLEDLights.getInstance();

        getArcingButton().whileTrue(Commands.either(
            lights.setBlinkingCommand(Color.kIndigo, Color.kBlue, 5.0), 
            lights.setAlingmentNoteStatusCommand(() -> !Feeder.getInstance().isNoteLoaded()), 
            () -> {
                System.out.println(Pivot.getInstance().isAtGoal() && Shooter.getInstance().getFlywheelIsAtVelocity());
                return Pivot.getInstance().isAtGoal() && Shooter.getInstance().getFlywheelIsAtVelocity();
            }
            ).repeatedly());
    }

    @Override
    public void registerCommands() {
        registerDrivetrain();
        registerLEDs();
    }
}
