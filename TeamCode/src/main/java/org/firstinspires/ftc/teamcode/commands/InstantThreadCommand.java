package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

public class InstantThreadCommand extends CommandBase {
    private final Runnable toRun;
    private boolean isFinished = false;

    public InstantThreadCommand(Runnable toRun) {
        this.toRun = toRun;
    }

    @Override
    public void initialize() {
        new Thread(toRun).start();
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}