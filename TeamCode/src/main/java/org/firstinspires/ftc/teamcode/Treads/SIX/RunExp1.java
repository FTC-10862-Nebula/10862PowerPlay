package org.firstinspires.ftc.teamcode.Treads.SIX;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class RunExp1 implements Runnable
{
    public RunExp1(DrivetrainCOrrect drivetrainCorrect, Slide slide, Arm arm) {
        new DriveForwardCommand(drivetrainCorrect, 10);
        new InstantCommand(slide::slideLow);
        new InstantCommand(arm::moveIntakeFAuto);
        run();

    }

    public void run()
    {
//        new DriveForwardCommand(drivetrain, 12);
        System.out.println("Thread is running...");
    }


    public static Command main(DrivetrainCOrrect drivetrainCorrect, Slide slide, Arm arm)
    {
        RunExp1 r1=new RunExp1(drivetrainCorrect, slide, arm);
        Thread thread1 =new Thread(r1);
        thread1.start();
        return null;
    }
}
