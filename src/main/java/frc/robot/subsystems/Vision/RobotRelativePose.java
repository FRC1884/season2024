package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotRelativePose extends Pose2d {

    /**
   * Child class of Pose2d that helps ensure we use the correct pose type for anything robot related, ie. aligning to an object.
   * @param translation translation component of the pose robot relative.
   * @param thetaRadians The rotational component of the pose robot relative.
   */
    public RobotRelativePose(Translation2d translation, Rotation2d thetaRadians){
        super(translation, thetaRadians);
    }
}
