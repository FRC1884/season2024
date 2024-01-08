package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ExampleFlywheel extends SubsystemBase {
  private static ExampleFlywheel instance;

  public static ExampleFlywheel getInstance() {
    if (instance == null) instance = new ExampleFlywheel();
    return instance;
  }

  // Motor Controllers
  private CANSparkMax leaderFlywheel, followerFlywheel;

  // PID Controllers and Gains
  // PID can be tuned in REV Hardware client
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private SparkMaxPIDController leader_pidController, follower_pidController;

  private ExampleFlywheel() {
    leaderFlywheel = new CANSparkMax(RobotMap.FlywheelMap.LEADER_FLYWHEEL, MotorType.kBrushless);
    followerFlywheel =
        new CANSparkMax(RobotMap.FlywheelMap.FOLLOWER_FLYWHEEL, MotorType.kBrushless);

    // This method is in case one of the motors needs to be invrted before setting them to follow
    leaderFlywheel.setInverted(false);
    followerFlywheel.follow(leaderFlywheel, false);

    leader_pidController = leaderFlywheel.getPIDController();
    follower_pidController = followerFlywheel.getPIDController();
    kP = 1;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMinOutput = 0;
    kMaxOutput = 0;

    leader_pidController.setP(kP);
    leader_pidController.setI(kI);
    leader_pidController.setD(kD);
    // leader_pidController.setIZone(kIz);
    // leader_pidController.setFF(kFF);
    // leader_pidController.setOutputRange(kMinOutput, kMaxOutput);

    follower_pidController.setI(kI);
    follower_pidController.setP(kP);
    follower_pidController.setD(kD);
    // follower_pidController.setIZone(kIz);
    // follower_pidController.setFF(kFF);
    // follower_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }
}
