package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;
import frc.robot.RobotMap.PivotMap;

public class DualActuatorPivot implements PivotHardware {
    private static DualActuatorPivot instance;

    public static DualActuatorPivot getInstance() {
        if (instance == null) instance = new DualActuatorPivot();
        return instance;
    }

    private final CANSparkFlex leader;
    private final CANSparkFlex follower;

    private final DutyCycleEncoder encoder;

    private DualActuatorPivot() {
        leader = new CANSparkFlex(RobotMap.DoubleActuatorPivotMap.PIVOT_ID_LEADER, MotorType.kBrushless);
        follower = new CANSparkFlex(RobotMap.DoubleActuatorPivotMap.PIVOT_ID_FOLLOWER, MotorType.kBrushless);

        leader.restoreFactoryDefaults();
        follower.restoreFactoryDefaults();

        leader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leader.setSmartCurrentLimit(30);
        follower.setSmartCurrentLimit(30);

        leader.burnFlash();
        follower.burnFlash();

        // follower.follow(leader, true);

        encoder = new DutyCycleEncoder(new DigitalInput(RobotMap.DoubleActuatorPivotMap.ENCODER_PORT));
    }

    public void zeroEncoder() {
        encoder.reset();
    }

    @Override
    public void setVoltage(double voltage) {
        leader.setVoltage(voltage);
        follower.setVoltage(-voltage);
    }

    @Override
    public double getEncoderPosition() {
        return encoder.get();
    }
}