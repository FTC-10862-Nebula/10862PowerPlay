package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideLowBCommand extends ParallelCommandGroup {
    public SlideLowBCommand(Slide slide, Arm arm, ClawServos clawServos, boolean auto) {
        if (auto){
            addCommands(
                    new InstantCommand(() ->
                            new Thread(() -> {
                                clawServos.clawClose();
                                slide.slideLow();
                                arm.moveBAuto();
                            }).start()),

                    new WaitCommand(200),
                    new InstantCommand(clawServos::setBClawPos)
            );
        }
        else {
            addCommands(
                    new InstantCommand(() ->
                            new Thread(() -> {
                                clawServos.clawClose();
                                slide.slideLow();
                                arm.moveB();
                            }).start()),

                    new WaitCommand(200),
                    new InstantCommand(clawServos::setBClawPos)


//                new InstantCommand(clawServos::clawClose),
//                new InstantCommand(slide::slideLow, slide),
//                new InstantCommand(arm::moveB, arm),
//                new WaitCommand(650),
//                new InstantCommand(clawServos::setBClawPos)
            );
        }
    }
}
