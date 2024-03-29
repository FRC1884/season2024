package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
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

    private final CANSparkMax leader;
    private final CANSparkMax follower;

    private final DutyCycleEncoder encoder;

    private DualActuatorPivot() {
        leader = new CANSparkMax(RobotMap.DoubleActuatorPivotMap.PIVOT_ID_LEADER, MotorType.kBrushless);
        follower = new CANSparkMax(RobotMap.DoubleActuatorPivotMap.PIVOT_ID_FOLLOWER, MotorType.kBrushless);

        leader.restoreFactoryDefaults();
        follower.restoreFactoryDefaults();

        leader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leader.setSmartCurrentLimit(40);
        follower.setSmartCurrentLimit(40);

        leader.burnFlash();
        follower.burnFlash();

        follower.follow(leader, true);

        // TODO: test if the soft limits work at all
        leader.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) PivotMap.UPPER_SETPOINT_LIMIT);
        leader.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) PivotMap.LOWER_SETPOINT_LIMIT);

        leader.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        leader.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        leader.burnFlash();
        follower.burnFlash();

        encoder = new DutyCycleEncoder(new DigitalInput(RobotMap.DoubleActuatorPivotMap.ENCODER_PORT));
    }

    public void zeroEncoder() {
        encoder.reset();
    }

    @Override
    public void setVoltage(double voltage) {
        leader.setVoltage(voltage);
    }

    @Override
    public double getEncoderPosition() {
        return encoder.get();
    }

    @Override
    public double getForwardLimit() {
        return leader.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward);
    }

    @Override
    public void setForwardLimit(double limit) {
        leader.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) limit);
    }

    @Override
    public double getReverseLimit() {
        return leader.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse);
    }

    @Override
    public void setReverseLimit(double limit) {
        leader.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) limit);
    }
}