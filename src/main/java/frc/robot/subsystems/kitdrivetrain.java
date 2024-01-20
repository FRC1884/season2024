package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class kitdrivetrain extends SubsystemBase {
    private static kitdrivetrain instance;
    private DifferentialDrive m_Drive;
    public static kitdrivetrain getInstance() {
        if (instance == null) instance = new kitdrivetrain();
        return instance;
    }
    private CANSparkMax leftFront;
    private CANSparkMax leftBack;
    private CANSparkMax rightFront;
    private CANSparkMax rightBack;

    private static final int deviceID = 1;

    //constructor 
    private kitdrivetrain(){
        leftFront = new CANSparkMax(deviceID, MotorType.kBrushless);
        leftBack.follow(leftFront);
        rightFront = new CANSparkMax(deviceID, MotorType.kBrushless);
        rightBack.follow(rightFront);
        rightFront.setInverted(true);

        DifferentialDrive m_Drive = new DifferentialDrive(leftFront, rightFront);
    }
    public void drive(double speed, double rotation){
        m_Drive.arcadeDrive(speed, rotation);

    }

    public Command driveCommand(Supplier<Double> spd, Supplier<Double> rot){
        return new RunCommand(() -> drive(spd.get(), rot.get()), this);
    
    }
}