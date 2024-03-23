package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ShooterMap;

public class Climber extends SubsystemBase {
    private static Climber instance;
    private ProfiledPIDController leaderProfile;
    private ProfiledPIDController followerProfile;

    public static Climber getInstance() {
        if(instance == null) instance = new Climber();
        return instance;
    }

    private CANSparkBase leaderMotor, followerMotor;

    private Climber() {
        setName("Climber");

        leaderMotor = new CANSparkMax(RobotMap.ClimberMap.MASTER_ID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(RobotMap.ClimberMap.SLAVE_ID, MotorType.kBrushless);
        var pidL = leaderMotor.getPIDController();
            pidL.setP(ShooterMap.FLYWHEEL_PID.kP);
            pidL.setI(ShooterMap.FLYWHEEL_PID.kI);
            pidL.setD(ShooterMap.FLYWHEEL_PID.kD);
            pidL.setFF(ShooterMap.FLYWHEEL_FF);
        var pidF = followerMotor.getPIDController();
            pidF.setP(ShooterMap.FLYWHEEL_PID.kP);
            pidF.setI(ShooterMap.FLYWHEEL_PID.kI);
            pidF.setD(ShooterMap.FLYWHEEL_PID.kD);
            pidF.setFF(ShooterMap.FLYWHEEL_FF);
        leaderMotor.restoreFactoryDefaults();
        followerMotor.restoreFactoryDefaults();

        leaderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        followerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        leaderMotor.setSmartCurrentLimit(30);
        followerMotor.setSmartCurrentLimit(30);

        // leaderMotor.setInverted(false);

        followerMotor.follow(leaderMotor, true);
        leaderMotor.burnFlash();
        followerMotor.burnFlash();
        //followerMotor.setInverted(true);

        var tab = Shuffleboard.getTab("Climber");

        tab.add(this);
    }

    public Command run(DoubleSupplier power) {
        return new InstantCommand(()->leaderMotor.set(power.getAsDouble()), this);
    }

    public SequentialCommandGroup alignSequence(){
        return new SequentialCommandGroup(positionUp(), new WaitCommand(5), positionDown()); // TODO: Change for time it takes to bring climber all the way up
    }

    public Command positionUp() {
        return Commands.runOnce(
            () -> {
                leaderProfile.setGoal(0); // TODO: Change value 0 to highest position the climber can reach
                followerProfile.setGoal(0);
            }
        ); // TODO: Change value 0 to highest position the climber can reach
    }
    public Command positionDown() {
        return Commands.runOnce(
            () -> {
                leaderProfile.setGoal(0); // TODO: Change value 0 to lowest position the climber can reach
                followerProfile.setGoal(0); // TODO: Change value 0 to lowest position the climber can reach
            }
        );
    }


    @Override
    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("Climber Power", () -> leaderMotor.get(), (s) -> leaderMotor.set(s));

    }
}
