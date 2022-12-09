package org.firstinspires.ftc.teamcode.Treads.thing;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ThreadComman;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class Drivethreadcomment extends SequentialCommandGroup{
//    private Thread;
    public Drivethreadcomment(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){

        new InstantCommand(() ->
                new Thread(() -> {
                    slide.slideLow();
                    arm.moveHighB();
                    clawServos.clawClose();
                }).start());
//       new Thread(
//                () -> {
////                    slide.slideLow();
//                    arm.moveHighB();
////                    new WaitCommand(12);
//                    clawServos.clawClose();
//                }
//        ).start();
//        Thread.currentThread().interrupt();
       return;

    }
}