package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

/**
 * A command that runs a Runnable continuously.  Has no end condition as-is;
 */
public class RunCommand extends CommandBase {
    protected final Runnable m_toRun;

    /**
     * Creates a new RunCommand.  The Runnable will be run continuously until the command
     * ends.  Does not run when disabled.
     *
     * @param toRun        the Runnable to run
     * @param requirements the subsystems to require
     */
    public RunCommand(Runnable toRun, Subsystem... requirements) {
        m_toRun = toRun;
        addRequirements(requirements);
    }

    @Override
    public void execute() {
        m_toRun.run();
    }
}

