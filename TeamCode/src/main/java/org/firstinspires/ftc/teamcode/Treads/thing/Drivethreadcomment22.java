package org.firstinspires.ftc.teamcode.Treads.thing;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class Drivethreadcomment22 extends SequentialCommandGroup{
//    private Thread;
    public Drivethreadcomment22(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){

        Thread one = new Thread(
                () -> {
                    slide.slideMid();
                    arm.moveIntakeB();
//                    new WaitCommand(12);
                    clawServos.clawClose();

                }

        );

        new Thread(one).start();
        return;
    }
}