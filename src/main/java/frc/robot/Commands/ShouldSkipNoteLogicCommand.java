package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.PoseEstimator;

// cleaned up for simplicity. 
// logic is continue with VI if a note was seen at the moment this command was scheduled, 
// stall out the rest of the auto otherwise
public class ShouldSkipNoteLogicCommand extends Command {
    Drivetrain drivetrain = Drivetrain.getInstance();
    Vision vision = Vision.getInstance();
    PoseEstimator poseEstimator = PoseEstimator.getInstance();
    boolean noteSeen;
    // final Pose2d startingPose;
    // Pose2d finalPose;

    public ShouldSkipNoteLogicCommand() {
        // startingPose = drivetrain.getPose();
    }

    @Override
    public void initialize() {
        poseEstimator.storeCurrentPose();
        noteSeen = vision.gamePieceDetected();
        // finalPose = vision.hasNoteInSight() ? vision.getNotePose2d() : startingPose;
    }

    @Override
    public boolean isFinished() {
        return !noteSeen; 
        // return startingPose.equals(finalPose);
    }

    @Override
    public void end(boolean interrupted) {
        // drivetrain.drive(0, 0, 0, false, false);
    }
}
