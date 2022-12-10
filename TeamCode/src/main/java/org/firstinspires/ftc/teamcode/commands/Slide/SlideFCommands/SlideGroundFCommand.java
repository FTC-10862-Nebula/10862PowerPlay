package org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideGroundFCommand extends SequentialCommandGroup {
    public SlideGroundFCommand(Slide slide, Arm arm, ClawServos clawServos, boolean auto) {
        if (auto){
            addCommands(
                    new InstantCommand(() ->
                            new Thread(() -> {
                                clawServos.clawClose();
                                slide.slideGround();
                                arm.moveFAuto();
                            }).start()),

                    new WaitCommand(200),
                    new InstantCommand(clawServos::setFClawPos)
            );
        }
        else {
            addCommands(
                    new InstantCommand(() ->
                            new Thread(() -> {
                                clawServos.clawClose();
                                slide.slideGround();
                                arm.moveF();
                            }).start()),

                    new WaitCommand(200),
                    new InstantCommand(clawServos::setFClawPos)


//                new InstantCommand(clawServos::clawClose),
//                new InstantCommand(clawServos::setFClawPos),
//                new InstantCommand(slide::slideGround, slide),
//                new WaitCommand(650),
//                new InstantCommand(arm::moveF, arm)
            );
        }

    }
}
