package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.PrototypeMap;

public class PrototypeTest extends SubsystemBase {

    private static PrototypeTest instance;

    public static PrototypeTest getInstance() {
        if (instance == null) instance = new PrototypeTest();
        return instance;
    }

    private CANSparkMax motor1, motor2, motor3;

    private ShuffleboardTab prototypeTab = Shuffleboard.getTab("Prototyping Tab");

    private PrototypeTest() {
        motor1 = new CANSparkMax(PrototypeMap.MOTOR_ID_1, MotorType.kBrushless);
        motor2 = new CANSparkMax(PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
        motor3 = new CANSparkMax(PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);

        prototypeTab.add("SparkMAX 1", new SendableMotor(motor1));
        prototypeTab.add("SparkMAX 2", new SendableMotor(motor2));
        prototypeTab.add("SparkMAX 3", new SendableMotor(motor3));
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}

class SendableMotor implements Sendable {
    private CANSparkMax m_motorController;
    private double setpoint;

    public SendableMotor(CANSparkMax motorController) {
        m_motorController = motorController;
    }

    void enable(boolean enabled) {
        if(!enabled) m_motorController.disable();
    }

    // adds a bunch of control properties on shuffleboard
    // do changes on the board affect behavior?
    // do I need to goof around in periodic?
    @Override
    public void initSendable(SendableBuilder builder) {
        SparkPIDController closedLoopController = m_motorController.getPIDController();
        builder.setActuator(true);

        builder.addBooleanProperty("Enabled", () -> m_motorController.get() == 0, this::enable);

        builder.addDoubleProperty("Speed", m_motorController::get, m_motorController::set);
        builder.addDoubleProperty("Setpoint",
                () -> setpoint,
                value -> {
                    setpoint = value;
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