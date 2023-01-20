package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideMidBCommand extends SequentialCommandGroup {
    public SlideMidBCommand(Slide slide, Arm arm, Claw claw, boolean auto) {
        if (auto){
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
                                        slide.slideMid();
                                        arm.moveBAuto();
                                    }).start())
                    ),
                    new WaitCommand(200),
                    new InstantCommand(claw::setBClawPos)
            );
        }
        else {
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
                                        slide.slideMid();
                                        arm.moveB();
                                    }).start())
                    ),
                    new WaitCommand(800),
                    new InstantCommand(claw::setBClawPos)
            );
        }
    }
}