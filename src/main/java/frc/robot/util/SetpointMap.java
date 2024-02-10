package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;

public class SetpointMap {
  private static SetpointMap instance;

  public static SetpointMap getInstance() {
    if (instance == null) {
      instance = new SetpointMap();
    }
    return instance;
  }

  // TODO add field positions
  public enum FieldPoses {
    SPEAKER_FAR_LEFT(1, 1, 1),
    SPEAKER_LEFT(1, 1, 1),
    SPEAKER_HEAD_ON(1, 1, 1),
    SPEAKER_RIGHT(1, 1, 1),
    SPEAKER_FAR_RIGHT(1, 1, 1),

    AMP(1, 1, 1),

    SOURCE(1, 1, 1);

    private Pose2d pose;

    FieldPoses(double x, double y, double rot) {
      pose = new Pose2d(x, y, new Rotation2d(rot));
    }

    Pose2d getPose() {
      return pose;
    }
  }

  private SetpointMap() {
    actionMap = new HashMap<String, ActionSetpoint>();
  }

  public void putSetpoint(String name, ActionSetpoint setpoint) {
    actionMap.put(name, setpoint);
  }

  private HashMap<String, ActionSetpoint> actionMap;

  public ActionSetpoint get(String key) {
    return actionMap.get(key);
  }
}
