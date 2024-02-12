package frc.robot.util;

import edu.wpi.first.wpilibj2.command.*;

/**
 * A command composition that runs a set of commands in parallel, ending when the specified
 * amount of time elapses.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 */
public final class RunForSecondsCommand extends ParallelDeadlineGroup {

    /**
     * Creates a new RunForSecondsCommand. The given commands will be executed simultaneously,
     * and will "race to the finish" - the first command to finish ends the entire command, with
     * all other commands being interrupted.
     * @param time the time (in seconds) that each command should run for
     * @param commands the command(s) to run as long as the timer is running. Commands that
     *                 complete instantly will NOT be rescheduled.
     */
    public RunForSecondsCommand(double time, Command... commands) {
        super(new WaitCommand(time));
        for (Command command : commands) addCommands(command);
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
