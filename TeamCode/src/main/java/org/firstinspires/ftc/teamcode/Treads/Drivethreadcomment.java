package org.firstinspires.ftc.teamcode.Treads;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

import java.util.concurrent.ThreadPoolExecutor;


public class Drivethreadcomment extends SequentialCommandGroup{
//    private Thread;
    public Drivethreadcomment(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
//        addCommands(
//                new DriveForwardCommand(drivetrain, -55),
//                new TurnToCommand(drivetrain, 48, true),
//                new SlideHighBAutoCommand(slide, arm, clawServos),
//                new WaitCommand(100),
//                new SlowDriveForwardCommand(drivetrain ,-8.1),
//                new WaitCommand(500),
//                new DropConeCommand(clawServos, slide, arm),
//                new WaitCommand(200),
//                new SlowDriveForwardCommand(drivetrain ,8),
//                new TurnToCommand(drivetrain, 90),
//                new SlideResetAutonFCommand(slide, arm, clawServos)
////                new InstantCommand(arm::moveReset, arm),
//        );
        new Thread(
                () -> {
                    slide.slideResting();
                    arm.moveHighB();
//                    new WaitCommand(12);
                    clawServos.clawClose();
                }
        );

        new Thread().start();
//        new ThreadGroup("teg");
//        new ThreadDeath();
//        new Thread().destroy();
//        new ThreadPoolExecutor()
//        new Thread().stop();
    }
}