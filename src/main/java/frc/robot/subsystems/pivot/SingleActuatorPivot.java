package frc.robot.subsystems.pivot;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Config;
import frc.robot.RobotMap;
import frc.robot.RobotMap.PivotMap;

public class SingleActuatorPivot implements PivotHardware {
    private static SingleActuatorPivot instance;

    public static SingleActuatorPivot getInstance() {
        if (instance == null) instance = new SingleActuatorPivot();
        return instance;
    }

    private CANSparkMax pivot;

    private SingleActuatorPivot() {
        pivot = new CANSparkMax(RobotMap.SingleActuatorPivotMap.PIVOT_ID, MotorType.kBrushless);

        pivot.restoreFactoryDefaults();
        pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivot.setSmartCurrentLimit(40);

        // TODO: test if the soft limits work at all
        pivot.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) PivotMap.UPPER_SETPOINT_LIMIT);
        pivot.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) PivotMap.LOWER_SETPOINT_LIMIT);

        pivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        pivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        pivot.burnFlash();
    }

    public void zeroEncoder() {
        pivot.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        pivot.setVoltage(voltage);
    }

    @Override
    public double getEncoderPosition() {
        return pivot.getEncoder().getPosition();
    }
}