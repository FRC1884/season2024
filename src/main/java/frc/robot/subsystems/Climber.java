package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
    private static Climber instance;

    public static Climber getInstance() {
        if(instance == null) instance = new Climber();
        return instance;
    }

    private CANSparkBase leaderMotor, followerMotor;

    private Climber() {
        leaderMotor = new CANSparkMax(RobotMap.ClimberMap.FOLLOWER, MotorType.kBrushless);
        followerMotor = new CANSparkMax(RobotMap.ClimberMap.LEADER, MotorType.kBrushless);

        // leaderMotor.setInverted(false);

        followerMotor.follow(leaderMotor, true);
        leaderMotor.burnFlash();
        followerMotor.burnFlash();
        //followerMotor.setInverted(true);
    }

    public Command run(DoubleSupplier power) {
        return new RunCommand(()->leaderMotor.set(power.getAsDouble()/5.0), this);
    }
}
