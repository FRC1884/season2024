package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotMap.PivotMap;

public class Pivot extends SubsystemBase {
    private static Pivot instance;

    public static Pivot getInstance() {
        if(instance == null) instance = new Pivot();
        return instance;
    }

    private ProfiledPIDController profiledPIDController;
    private CANSparkMax motor;
    private TrapezoidProfile.State goalState;

    private Pivot() {
        motor = new CANSparkMax(PivotMap.PIVOT, MotorType.kBrushless);
        profiledPIDController = new ProfiledPIDController(
                PivotMap.PID.getP(),
                PivotMap.PID.getI(),
                PivotMap.PID.getD(),
                PivotMap.PROFILE_CONSTRAINTS
        );
        profiledPIDController.reset(0.0);
        Shuffleboard.getTab("Pivot").add("goal pos", goalState.position);
        Shuffleboard.getTab("Pivot").add("goal vel", goalState.velocity);
    }

    public Command moveTo(double ticks) {
        return new ProfiledPIDCommand(
            profiledPIDController,
            ()->motor.getEncoder().getPosition(),
            () -> ticks,
            (vel, state) -> {
                motor.set(vel);
                goalState = state;
            },
            this
        );
    }
}