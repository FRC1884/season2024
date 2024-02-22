package frc.robot.Commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AddressableLEDLights;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.subsystems.Feeder.FeederDirection;

public class IntakeUntilLoadedCommand extends Command{
    Feeder feeder = Feeder.getInstance();
    Intake intake = Intake.getInstance();

    @Override
    public void initialize(){
        intake.setIntakeState(IntakeDirection.FORWARD);
        feeder.setFeederState(FeederDirection.FORWARD);
        System.out.println("Hell0");
    }
    
    @Override
    public void end(boolean interrupted){
        intake.setIntakeState(IntakeDirection.STOPPED);
        feeder.setFeederState(FeederDirection.STOPPED);
        System.out.println("Command Ended");
    }

    @Override
    public boolean isFinished(){
        return feeder.isNoteLoaded();
    }
}
