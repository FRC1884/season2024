package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class test extends SubsystemBase{

    private static test instance;
    public static test getInstance(){
        if(instance == null)
        instance = new test();
        return instance;
    }
    

    private test(){}

    public Command testCommand(Supplier<Double> speed){
        return new RepeatCommand(new RunCommand(() -> System.out.println(speed.get()), this));
    }
}
