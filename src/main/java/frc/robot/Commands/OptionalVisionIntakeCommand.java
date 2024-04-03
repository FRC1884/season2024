package frc.robot.Commands;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.PoseEstimator;

public class OptionalVisionIntakeCommand extends Command {
    Drivetrain drivetrain = Drivetrain.getInstance();
    PoseEstimator poseEstimator = PoseEstimator.getInstance();
    Vision vision = Vision.getInstance();
    Feeder feeder = Feeder.getInstance();
    Intake intake = Intake.getInstance();
    
    public OptionalVisionIntakeCommand() {
        addRequirements(drivetrain, poseEstimator, vision, feeder);
    }

    // need requirements??
    public void initialize() {
        Supplier<Pose2d> temp = poseEstimator.storeCurrentPose();

        
        // for(int i = 0; i < 100; i++) {
        //     System.out.println(poseEstimator.getStoredPose().get().toString());
        // }
        // for(int i = 0; i < 100; i++) {
        //     System.out.println(temp.get().toString());
        // }
            // () -> new IntakeUntilLoadedCommand().alongWith(
            // new ShouldSkipNoteLogicCommand().raceWith(vision.PIDtoNoteRobotRelativeCommand(feeder::isNoteLoaded)))
            // .andThen(drivetrain.chasePoseCommand(temp)), Set.of(drivetrain, poseEstimator, vision, feeder));
        // System.out.println(poseEstimator.getStoredPose().toString());

        addRequirements(drivetrain, feeder, intake);
    }

    public boolean isFinished() {
        return false;
    }
}
