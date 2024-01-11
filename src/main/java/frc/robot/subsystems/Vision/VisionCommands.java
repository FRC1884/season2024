import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap.VisionConfig;

public class VisionCommands {
    //Aiming should be reworked to instead provide key data values for on the fly path generation using pathplanner
    //Link: https://pathplanner.dev/pplib-create-a-path-on-the-fly.html
    //Particularly the projected Pose2d of the robot in relation to the target AprilTag or Note

    /*public static Command aimToHybridSpot(int spot) {
        return PilotCommands.aimPilotDrive(
                        () ->
                                Robot.pose.getHeading().getRadians()
                                        + Units.degreesToRadians(
                                                Robot.vision.getThetaToHybrid(
                                                        spot))) // or Robot.swerve.getRotation()?
                .withName("Aim to Hybrid Spot");
    }

    public static Command printYawInfo() {
        return new InstantCommand(
                () ->
                        RobotTelemetry.print(
                                "Yaw (D): "
                                        + Robot.vision.photonVision.getYaw()
                                        + "|| gyro (D): "
                                        + Robot.swerve.getRotation().getDegrees()
                                        + " || Aiming at: "
                                        + (Robot.vision.photonVision.getYaw()
                                                + Robot.swerve.getRotation().getDegrees())));
    }

    public static Command printEstimatedPhotonPoseInfo() {
        if (Robot.vision.photonVision.currentPose != null) {
            Pair<Pose3d, Double> pose = Robot.vision.photonVision.currentPose;
            return new InstantCommand(
                    () ->
                            RobotTelemetry.print(
                                    "Estimated Pose: | X: "
                                            + pose.getFirst().getTranslation().getX()
                                            + " | Y: "
                                            + pose.getFirst().getTranslation().getY()
                                            + " | Z: "
                                            + pose.getFirst().getTranslation().getZ()
                                            + " | Rotation (D): "
                                            + Units.radiansToDegrees(
                                                    pose.getFirst().getRotation().getZ())
                                            + " | Latency: "
                                            + pose.getSecond().doubleValue()));
        } else {
            return new InstantCommand(
                    () -> RobotTelemetry.print("PhotonVision doesn't have a pose!"));
        }
    }
    */

    /**
     * @param limelight name of limelight to control in {@link VisionConfig}
     * @param pipelineIndex use pipeline indexes in {@link VisionConfig}
     */
    public static Command setLimelightPipeline(String limelight, int pipelineIndex) {
        return new InstantCommand(
                        () -> Robot.vision.setLimelightPipeline(limelight, pipelineIndex),
                        Robot.vision)
                .ignoringDisable(true);
    }

    public static Command setCubeNodePipeline() {
        return setLimelightPipeline("", VisionConfig.aprilTagPipeline);
    }

    public static Command setConeNodePipeline() {
        return setLimelightPipeline("", VisionConfig.reflectivePipeline);
    }

    public static Command setConeDetectPipeline() {
        return setLimelightPipeline("", VisionConfig.coneDetectorPipeline);
    }

    public static Command setCubeDetectPipeline() {
        return setLimelightPipeline("", VisionConfig.cubeDetectorPipeline);
    }
}