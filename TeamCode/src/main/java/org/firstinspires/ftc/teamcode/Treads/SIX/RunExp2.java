package org.firstinspires.ftc.teamcode.Treads.SIX;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class RunExp2 implements Runnable
{
    public RunExp2(Drivetrain drivetrain, Slide slide, Arm arm) {
//        new DriveForwardCommand(drivetrain, 10);
        new InstantCommand(slide::slideHigh);
        new InstantCommand(arm::moveIntakeB);
        run();

    }

    public void run()
    {
//        new DriveForwardCommand(drivetrain, 12);
        System.out.println("Thread 2 is running...");
    }


    public static Command main(Drivetrain drivetrain, Slide slide, Arm arm)
    {
        Thread thread1 =new Thread(new RunExp2(drivetrain, slide, arm));
        thread1.start();
        return null;
    }
}
