package frc.robot;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.selector.AutoModeSelector;
import frc.robot.core.util.CTREConfigs;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision.PoseEstimator;
import frc.robot.subsystems.Vision.Vision;
// import frc.robot.subsystems.PrototypeSubsystem;
import frc.robot.util.SendableMotor;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;
  CANSparkBase motor1, motor2, motor3, motor4;
  SendableMotor motor1Sendable, motor2Sendable, motor3Sendable, motor4Sendable;
  private final Field2d m_field = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // TODO put auto chooser here. make sure to use the one from
    // robot/auto/selector/AutoModeSelector.java
    ctreConfigs = new CTREConfigs();

    enableLiveWindowInTest(true);
    var autoModeSelector = AutoModeSelector.getInstance();
    OI.getInstance();
    SmartDashboard.putData("field", m_field);
    // motor1 =
    //     new CANSparkFlex(
    //         RobotMap.PrototypeMap.MOTOR_ID_1,
    //         MotorType.kBrushless); // TODO: Make sure that it is the right Motor
    // motor2 = new CANSparkFlex(RobotMap.PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
    // // motor3 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
    // // motor4 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_4, MotorType.kBrushless);

    // motor1Sendable = new SendableMotor(motor1);
    // motor2Sendable = new SendableMotor(motor2);

    // SendableRegistry.addLW(motor1Sendable, "Prototype", "Motor 1");
    // SendableRegistry.addLW(motor2Sendable, "Prototype", "Motor 2");
    // SendableRegistry.addLW(new SendableMotor(motor3), "Prototype", "Motor 3");
    // SendableRegistry.addLW(new SendableMotor(motor4), "Prototype", "Motor 4");
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
    // UNTESTED GLASS TELEMETRY CODE - MAY RESULTS IN NULL POINTS
    m_field.getObject("Odometry Pose").setPose(Drivetrain.getInstance().getPose());
    if (Vision.getInstance().visionBotPose() != null){
      m_field.getObject("Vision Pose").setPose(Vision.getInstance().visionBotPose());
    }
    m_field.getObject("PoseEstimate Pose").setPose(PoseEstimator.getInstance().getPosition());
  
    // if (motor1Sendable.openLoopEnabled) motor1.set(motor1Sendable.m_speed);
    // else motor1.set(0.0);
    // if (motor2Sendable.openLoopEnabled) motor2.set(motor2Sendable.m_speed);
    // else motor2.set(0.0);
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
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    OI.getInstance().registerCommands();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // motor1 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_1, MotorType.kBrushless);
    // motor2 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
    // motor3 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
    // motor4 = new CANSparkMax(RobotMap.PrototypeMap.MOTOR_ID_4, MotorType.kBrushless);

    // SendableRegistry.addLW(new SendableMotor(motor1), "Prototype", "Motor 1");
    // SendableRegistry.addLW(new SendableMotor(motor2), "Prototype", "Motor 2");
    // SendableRegistry.addLW(new SendableMotor(motor3), "Prototype", "Motor 3");
    // SendableRegistry.addLW(new SendableMotor(motor4), "Prototype", "Motor 4");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // if (motor1Sendable.openLoopEnabled) motor1.set(motor1Sendable.m_speed);
    // else motor1.set(0.0);

    // if (motor2Sendable.openLoopEnabled) motor2.set(motor2Sendable.m_speed);
    // else motor2.set(0.0);

    // if (motor3Sendable.openLoopEnabled) motor3.set(motor3Sendable.m_speed);
    // else motor3.set(0.0);

    // if (motor4Sendable.openLoopEnabled) motor4.set(motor4Sendable.m_speed);
    // else motor4.set(0.0);

    /*ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    GenericEntry shooterEnable = tab.add("Shooter Enable", false).getEntry();

    // Command Example assumed to be in a PIDSubsystem
    new NetworkButton((BooleanSubscriber) shooterEnable).onTrue(new InstantCommand(PrototypeSubsystem.getInstance()::enable));

    // Timed Robot Example
    if (shooterEnable.getBoolean(false)) {
        // Calculates the output of the PID algorithm based on the sensor reading
        // and sends it to a motor
        PrototypeSubsystem.getInstance().runTo(1.0)
                .onlyIf(() -> shooterEnable.getBoolean(false));
    }*/
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
