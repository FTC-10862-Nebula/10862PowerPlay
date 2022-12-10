package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;

public class ThreadCommand extends CommandBase {

    private final CommandBase command;
//    private final CommandBase commandtwo;
    public ThreadCommand(CommandBase command) {
        this.command = command;
    }

//    public ThreadCommand(CommandBase command, CommandBase commandtwo) {
//        this.command = command;
//        this.commandtwo = commandtwo;
//    }

    @Override
    public void initialize() {
        new Thread(command::initialize).start();
    }

//    @Override
//    public void execute() {
//        new Thread(command::execute).start();
//    }

//    @Override
//    public void end(boolean interrupted) {
//        new Thread(() -> command.end(interrupted)).start();
//    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}