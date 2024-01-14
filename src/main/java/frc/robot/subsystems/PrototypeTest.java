package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.PrototypeMap;

import java.util.Objects;

/**
 * The prototyping subsystem. Currently, it has support for three SparkMAX controllers.
 * This means you can test with both NEO 550s and Vortexes. In the future,
 * it might be worth going all the way up to
 * {@link edu.wpi.first.wpilibj.motorcontrol.MotorController}
 * to facilitate Krakens and Falcons, too.
 * <br><br>
 * For now, this subsystem gives us the means to run mechanisms containing up to three motors
 * with dashboard-tunable parameters such as speed and feedback gains, allowing us to visualize
 * their actual performance during competition.
 * <br><br>
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
        if (Objects.equals(MOTOR_1_CLASS, CANSparkMax.class))
            m_motor1 = new CANSparkMax(PrototypeMap.MOTOR_ID_1, MotorType.kBrushless);
        else if (Objects.equals(MOTOR_1_CLASS, CANSparkFlex.class))
            m_motor1 = new CANSparkFlex(PrototypeMap.MOTOR_ID_1, MotorType.kBrushless);
        else throw new RuntimeException();

        if (Objects.equals(MOTOR_2_CLASS, CANSparkMax.class))
            m_motor2 = new CANSparkMax(PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
        else if (Objects.equals(MOTOR_2_CLASS, CANSparkFlex.class))
            m_motor2 = new CANSparkFlex(PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
        else throw new RuntimeException();

        if (Objects.equals(MOTOR_3_CLASS, CANSparkMax.class))
            m_motor3 = new CANSparkMax(PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
        else if (Objects.equals(MOTOR_3_CLASS, CANSparkFlex.class))
            m_motor3 = new CANSparkFlex(PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
        else throw new RuntimeException();

        prototypeTab.add("Motor 1", new SendableMotor(m_motor1));
        prototypeTab.add("Motor 2", new SendableMotor(m_motor2));
        prototypeTab.add("Motor 3", new SendableMotor(m_motor3));
    }

    // This being entirely empty is concerning...
    // Oh well.
    @Override
    public void periodic() {
        super.periodic();
    }
}

/**
 * The {@link Sendable} implementation for motors that allows us to send an entire motor
 * as a package to Shuffleboard or Elastic.
 * <br><br>
 * <b>For this to work, you MUST run the program in Test Mode on the Driver Station!</b>
 */
class SendableMotor implements Sendable {
    private CANSparkBase m_motorController;
    private double m_setpoint;

    public SendableMotor(CANSparkBase motorController) {
        m_motorController = motorController;
    }

    void enable(boolean enabled) {
        if(!enabled) m_motorController.disable();
    }

    // Adds a bunch of control properties on shuffleboard
    // Do changes on the board affect behavior?
    // Do I need to goof around in periodic?
    // Only time will tell...
    @Override
    public void initSendable(SendableBuilder builder) {
        SparkPIDController closedLoopController = m_motorController.getPIDController();
        builder.setActuator(true);

        builder.addBooleanProperty("Enabled", () -> m_motorController.get() == 0, this::enable);

        builder.addDoubleProperty("Speed", m_motorController::get, m_motorController::set);
        builder.addDoubleProperty("Setpoint",
                () -> m_setpoint,
                value -> {
                    m_setpoint = value;
                    closedLoopController.setReference(value, ControlType.kSmartMotion);
                });

        builder.addDoubleProperty("P", closedLoopController::getP, closedLoopController::setP);
        builder.addDoubleProperty("I", closedLoopController::getI, closedLoopController::setI);
        builder.addDoubleProperty("D", closedLoopController::getD, closedLoopController::setD);
        builder.addDoubleProperty("FF", closedLoopController::getFF, closedLoopController::setFF);
        builder.addDoubleProperty("IZone", closedLoopController::getIZone, closedLoopController::setIZone);

        int id = m_motorController.getDeviceId();

        builder.addDoubleProperty("MaxVel",
                () -> closedLoopController.getSmartMotionMaxVelocity(id),
                maxVel -> closedLoopController.setSmartMotionMaxVelocity(maxVel, id));
        builder.addDoubleProperty("MaxAccel",
                () -> closedLoopController.getSmartMotionMaxAccel(id),
                maxAccel -> closedLoopController.setSmartMotionMaxAccel(maxAccel, id));
        builder.addDoubleProperty("AllowedErr",
                () -> closedLoopController.getSmartMotionAllowedClosedLoopError(id),
                allowedErr -> closedLoopController.setSmartMotionAllowedClosedLoopError(allowedErr, id));
    }
}