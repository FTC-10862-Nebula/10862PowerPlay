package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideMidBackCommand extends SequentialCommandGroup {
    public SlideMidBackCommand(Slide slide, Arm arm, ClawServos clawServos) {
//        new Thread(
//                () -> {
//                    clawServos.clawClose();
//                    slide.slideMid();
//                    arm.moveB();
//                    new WaitCommand(650);
//                    clawServos.setBClawPos();
//                }
//        ).start();
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(slide::slideMid, slide),
                new InstantCommand(arm::moveB, arm),
                new WaitCommand(650),
                new InstantCommand(clawServos::setBClawPos)
        );
    }
}