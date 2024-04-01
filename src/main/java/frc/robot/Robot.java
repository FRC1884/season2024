package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.core.util.CTREConfigs;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.PhotonPoseTracker;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.RobotMap.VisionConfig;
import frc.robot.auto.AutoCommands;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  private final Field2d m_field = new Field2d();
  private SendableChooser<Command> autoChooser;
  private ShuffleboardTab tab = Shuffleboard.getTab("Odometry Data");
  private GenericEntry visionFilterEntry = tab.add("Vision Filter Overide", false).getEntry();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // TODO put auto chooser here. make sure to use the one from
    // robot/auto/selector/AutoModeSelector.java
    
    //OI.getInstance();

    //Autocommands
    OI.getInstance().registerCommands();
    AutoCommands.registerAutoCommands();
    ctreConfigs = new CTREConfigs();

    if(RobotMap.PrototypeMap.LIVE_WINDOW_ENABLED)
      enableLiveWindowInTest(true);

    SmartDashboard.putData("field", m_field);
    
    if(Config.Subsystems.DRIVETRAIN_ENABLED){
       //Zero Gyro
      Drivetrain.getInstance().zeroGyroYaw();
      PoseEstimator.getInstance().resetPoseEstimate(Drivetrain.getInstance().getPose()); 
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //PoseEstimator.getInstance().setEstimatedPose(Drivetrain.getInstance().getPose());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if(Config.Subsystems.DRIVETRAIN_ENABLED) {
      m_field.getObject("Odometry Pose").setPose(Drivetrain.getInstance().getPose());
      m_field.getObject("Vision Pose").setPose(Vision.getInstance().visionBotPose());
      m_field.getObject("PoseEstimate Pose").setPose(PoseEstimator.getInstance().getPosition());
      
      // Vision Poses with New Filtering
      ArrayList<PhotonPoseTracker> trackers = Vision.getInstance().getPhotonPoseTrackers();
      if(VisionConfig.IS_PHOTON_VISION_ENABLED) m_field.getObject("Vision Cam 1 Pose").setPose(trackers.get(0).getEstimatedVisionBotPose());
      if(VisionConfig.IS_PHOTON_TWO_ENABLED) m_field.getObject("Vision Cam 2 Pose").setPose(trackers.get(1).getEstimatedVisionBotPose());
      if(VisionConfig.IS_PHOTON_THREE_ENABLED) m_field.getObject("Vision Cam 3 Pose").setPose(trackers.get(2).getEstimatedVisionBotPose());
    }  
    if (Vision.getInstance().getNotePose2d() != null){
      m_field.getObject("Note Pose").setPose(Vision.getInstance().getNotePose2d());
      
    }


    if (visionFilterEntry.getBoolean(false)){
      m_field.getObject("Filtered Vision Pose").setPose(Vision.getInstance().getFilterVisionPose());
    }
    
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_field.getObject("path").setPoses(poses);
    });
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    var autonomousCommand = getAutonomousCommand();

      // Supplier<Pose2d> getTarget = () -> DriverStation.getAlliance().isPresent()
      // && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
      // ? RobotMap.Coordinates.BLUE_SPEAKER
      // : RobotMap.Coordinates.RED_SPEAKER;

      if(autonomousCommand != null) {
        autonomousCommand.schedule();
        // if(Config.Subsystems.DRIVETRAIN_ENABLED && Config.Subsystems.PIVOT_ENABLED) {
        //   Pivot pivot = Pivot.getInstance();
        //   pivot.setDefaultCommand(pivot.updatePosition(
        //     () -> RobotMap.ShooterMap.SPEAKER_LOOKUP_TABLE.get(PoseEstimator.getInstance().getDistanceToPose(getTarget.get().getTranslation()))
        //       .getAngle()));
        // }
        // if(Config.Subsystems.DRIVETRAIN_ENABLED && Config.Subsystems.SHOOTER_ENABLED) {
        //   Shooter shooter = Shooter.getInstance();
        //   shooter.setDefaultCommand(shooter.setFlywheelVelocityCommand(
        //     () -> RobotMap.ShooterMap.SPEAKER_LOOKUP_TABLE.get(PoseEstimator.getInstance().getDistanceToPose(getTarget.get().getTranslation()))
        //       .getRPM()));
        // }
      }
    //Drivetrain.getInstance().setGyroYaw(180);
  }
    

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
