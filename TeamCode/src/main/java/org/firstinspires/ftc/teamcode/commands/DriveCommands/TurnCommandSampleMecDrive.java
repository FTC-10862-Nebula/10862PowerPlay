package org.firstinspires.ftc.teamcode.commands.DriveCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class TurnCommandSampleMecDrive extends CommandBase {

    private final SampleMecanumDrive mecanumDrive;
    private final double angle;

    //Turns in a Counterclockwise direction
    public TurnCommandSampleMecDrive(SampleMecanumDrive mecanumDrive, double angle) {
        this.mecanumDrive = mecanumDrive;
        this.angle = angle;

        addRequirements(mecanumDrive);
    }

    @Override
    public void initialize() {
        mecanumDrive.turn(Math.toRadians(angle));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            mecanumDrive.stop();
        }
    }

    @Override
    public void execute() {
        mecanumDrive.update();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !mecanumDrive.isBusy();
    }

}