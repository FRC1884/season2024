package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The prototyping subsystem. Currently, it has support for three SparkMAX controllers. This means
 * you can test with both NEO 550s and Vortexes. In the future, it might be worth going all the way
 * up to {@link edu.wpi.first.wpilibj.motorcontrol.MotorController} to facilitate Krakens and
 * Falcons, too. <br>
 * <br>
 * For now, this subsystem gives us the means to run mechanisms containing up to three motors with
 * dashboard-tunable parameters such as speed and feedback gains, allowing us to visualize their
 * actual performance during competition. <br>
 * <br>
 * <b>To tweak these parameters, you MUST run the project in Test Mode on the Driver Station!</b>
 */
public class PrototypeTest extends SubsystemBase {

  private static PrototypeTest instance;

  public static PrototypeTest getInstance() {
    if (instance == null) instance = new PrototypeTest();
    return instance;
  }

  private CANSparkBase m_motor1, m_motor2, m_motor3;

  private ShuffleboardTab prototypeTab = Shuffleboard.getTab("Prototyping Tab");

  // Credit to @RBrott and the RoadRunner Quickstart for this idea!
  private static final Class<?> MOTOR_1_CLASS = CANSparkMax.class;
  private static final Class<?> MOTOR_2_CLASS = CANSparkMax.class;
  private static final Class<?> MOTOR_3_CLASS = CANSparkMax.class;

  private PrototypeTest() {
    //        if (Objects.equals(MOTOR_1_CLASS, CANSparkMax.class))
    //            m_motor1 = new CANSparkMax(PrototypeMap.MOTOR_ID_1, MotorType.kBrushless);
    //        else if (Objects.equals(MOTOR_1_CLASS, CANSparkFlex.class))
    //            m_motor1 = new CANSparkFlex(PrototypeMap.MOTOR_ID_1, MotorType.kBrushless);
    //        else throw new RuntimeException();
    //
    //        if (Objects.equals(MOTOR_2_CLASS, CANSparkMax.class))
    //            m_motor2 = new CANSparkMax(PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
    //        else if (Objects.equals(MOTOR_2_CLASS, CANSparkFlex.class))
    //            m_motor2 = new CANSparkFlex(PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
    //        else throw new RuntimeException();
    //
    //        if (Objects.equals(MOTOR_3_CLASS, CANSparkMax.class))
    //            m_motor3 = new CANSparkMax(PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
    //        else if (Objects.equals(MOTOR_3_CLASS, CANSparkFlex.class))
    //            m_motor3 = new CANSparkFlex(PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
    //        else throw new RuntimeException();
    //
    //        prototypeTab.add("Motor 1", new SendableMotor(m_motor1));
    //        prototypeTab.add("Motor 2", new SendableMotor(m_motor2));
    //        prototypeTab.add("Motor 3", new SendableMotor(m_motor3));
  }

  // This being entirely empty is concerning...
  // Oh well.
  @Override
  public void periodic() {
    super.periodic();
  }
}
