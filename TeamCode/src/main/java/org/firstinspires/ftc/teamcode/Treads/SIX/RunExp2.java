package org.firstinspires.ftc.teamcode.Treads.SIX;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class RunExp2 implements Runnable
{
    public RunExp2(DrivetrainCOrrect drivetrainCorrect, Slide slide, Arm arm) {
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


    public static Command main(DrivetrainCOrrect drivetrainCorrect, Slide slide, Arm arm)
    {
        RunExp2 r2=new RunExp2(drivetrainCorrect, slide, arm);
        Thread thread1 =new Thread(r2);
        thread1.start();
        return null;
    }
}
