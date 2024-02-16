package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.subsystems.Shooter.FeederDirection;

public class IntakeUntilLoadedCommand extends Command{
    Shooter shooter = Shooter.getInstance();
    Intake intake = Intake.getInstance();

    @Override
    public void initialize(){
        intake.setIntakeState(IntakeDirection.STOPPED);
        shooter.setFeederState(FeederDirection.FORWARD);
    }
    
    @Override
    public void end(boolean interrupted){
        intake.setIntakeState(IntakeDirection.STOPPED);
        shooter.setFeederState(FeederDirection.STOPPED);
    }

    @Override
    public boolean isFinished(){
        return shooter.isNoteLoaded();
    }
}
