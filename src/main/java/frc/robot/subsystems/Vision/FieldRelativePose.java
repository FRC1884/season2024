package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldRelativePose extends Pose2d {
    
    /**
   * Child class of Pose2d that helps ensure we use the correct pose type for anything field related, ie. pathplanner.
   * @param translation translation component of the pose field relative.
   * @param thetaRadians The rotational component of the pose field relative.
   */
    public FieldRelativePose(Translation2d translation, Rotation2d thetaRadians){
        super(translation, thetaRadians);
    }

}
