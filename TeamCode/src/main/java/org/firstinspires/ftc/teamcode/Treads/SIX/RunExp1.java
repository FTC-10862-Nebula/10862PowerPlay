package org.firstinspires.ftc.teamcode.Treads.SIX;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class RunExp1 implements Runnable
{
    public RunExp1(Drivetrain drivetrain, Slide slide, Arm arm) {
//        new DriveForwardCommand(drivetrain, -10);
        new InstantCommand(slide::slideLow);
        new InstantCommand(arm::moveIntakeFAuto);
        run();

    }

    public void run()
    {
        System.out.println("Thread is running RunExp1...");
    }


    public static Command main(Drivetrain drivetrain, Slide slide, Arm arm)
    {
        Thread thread1 =new Thread(new RunExp1(drivetrain, slide, arm));
        thread1.start();
        return null;
    }
}
