package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Config;
import frc.robot.Commands.IntakeUntilLoadedCommand;
import frc.robot.RobotMap.Coordinates;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Feeder.FeederDirection;
import frc.robot.util.FlywheelLookupTable;
import frc.robot.subsystems.PoseEstimator;

import java.sql.Driver;
import java.util.function.Supplier;

public class AutoCommands {
        public static void registerAutoCommands() {
                FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();

                // default to red speaker to avoid null pointer exception
                Supplier<Pose2d> getTarget = () -> DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                                                ? Coordinates.BLUE_SPEAKER
                                                : Coordinates.RED_SPEAKER;

                PoseEstimator poseEstimator = PoseEstimator.getInstance();
                Pivot pivot = Pivot.getInstance();
                Shooter shooter = Shooter.getInstance();
                Feeder feeder = Feeder.getInstance();

                NamedCommands.registerCommand("Intake", new IntakeUntilLoadedCommand());

                NamedCommands.registerCommand("SpoolShooter", shooter.setFlywheelVelocityCommand(
                                () -> lookupTable.get(poseEstimator.getDistanceToPose(getTarget.get().getTranslation()))
                                                .getRPM()));

                NamedCommands.registerCommand("Pivot", pivot.updatePosition(
                                () -> lookupTable.get(poseEstimator.getDistanceToPose(getTarget.get().getTranslation()))
                                                .getAngle()));

                NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
                                new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD),
                                                Feeder.getInstance()),
                                new WaitCommand(0.3),
                                new InstantCommand(() -> feeder.setFeederState(FeederDirection.STOPPED),
                                                Feeder.getInstance())));

                NamedCommands.registerCommand("VisionIntake", new IntakeUntilLoadedCommand()
                                .raceWith(Vision.getInstance().PIDtoNoteRobotRelativeCommand()));
                // NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));
                // NamedCommands.registerCommand("SpoolShooter", new PrintCommand("Spooling"));
                // NamedCommands.registerCommand("Shoot", new PrintCommand("Shoot"));
                // NamedCommands.registerCommand("StopShooter", new PrintCommand("Stop
                // Spooling"));
        }
}
