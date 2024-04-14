package frc.robot.Commands;


import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.PoseEstimator;

public class VisionIntakeThenReturnCommand extends SequentialCommandGroup{

    Drivetrain drivetrain = Drivetrain.getInstance();
    PoseEstimator poseEstimator = PoseEstimator.getInstance();
    Vision vision = Vision.getInstance();
    Feeder feeder = Feeder.getInstance();
    Intake intake = Intake.getInstance();

    //Supplier<Pose2d> initialPose = () -> new Pose2d();

    private Set<Subsystem> reqSubs = new HashSet<Subsystem>();
    public VisionIntakeThenReturnCommand(){

        addRequiredSubsystems();
        addCommands(
            new InstantCommand( () -> poseEstimator.storeCurrentPose()),
            
            new IntakeUntilLoadedCommand().alongWith(new ShouldSkipNoteLogicCommand().raceWith(vision.PIDtoNoteRobotRelativeCommand(feeder::isNoteLoaded))),
            drivetrain.chasePoseCommand(poseEstimator.getStoredPose())
        );
    }

    private void addRequiredSubsystems(){
        reqSubs.add(drivetrain);
        reqSubs.add(feeder);
        reqSubs.add(intake);
        reqSubs.add(vision);
        reqSubs.add(poseEstimator);
    }
    public Set<Subsystem> getRequiredSubsystems(){
        return reqSubs;
    }
    
}
