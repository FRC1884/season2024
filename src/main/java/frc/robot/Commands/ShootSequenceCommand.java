package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap.Coordinates;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.subsystems.Intake.IntakeStatus;
import frc.robot.subsystems.Feeder.FeederDirection;
import frc.robot.util.FlywheelLookupTable;

public class ShootSequenceCommand extends SequentialCommandGroup {
    Shooter shooter = Shooter.getInstance();
    Feeder feeder = Feeder.getInstance();
    Intake intake = Intake.getInstance();
    Pivot pivot = Pivot.getInstance();
    PoseEstimator poseEstimator = PoseEstimator.getInstance();
    FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();
    Pose2d target = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;

    public ShootSequenceCommand() {
        addCommands(
            // new ParallelCommandGroup(
            //     new RunCommand(() -> pivot.setPosition(lookupTable
            //                     .get(poseEstimator.getDistanceToPose(target.getTranslation())).getAngleSetpoint()))
            //                     .until(() -> pivot.isAtGoal()),
                
            //     shooter.setFlywheelVelocityCommand(lookupTable.get(
            //         poseEstimator.getDistanceToPose(target.getTranslation())).getRPM()).until(() -> shooter.getFlywheelIsAtVelocity())
            // // ),
            // shooter.setFlywheelVelocityCommand(lookupTable.get(
            //         poseEstimator.getDistanceToPose(target.getTranslation())).getRPM()).until(() -> shooter.getFlywheelIsAtVelocity()),
            new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD)).onlyWhile(() -> feeder.isNoteLoaded()),
            new InstantCommand(() -> shooter.stopFlywheelCommand()).onlyIf(() -> !feeder.isNoteLoaded())
        );
    }
}
