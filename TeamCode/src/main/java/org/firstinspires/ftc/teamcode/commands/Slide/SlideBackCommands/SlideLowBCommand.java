package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideLowBCommand extends SequentialCommandGroup {
    public SlideLowBCommand(Slide slide, Arm arm, ClawServos clawServos, boolean auto) {
        if (auto){
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        clawServos.clawClose();
                                        slide.slideLow();
                                        arm.moveBAuto();
                                    }).start())
                    ),
                    new WaitCommand(200),
                    new InstantCommand(clawServos::setBClawPos)
            );
        }
        else {
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        clawServos.clawClose();
                                        slide.slideLow();
                                        arm.moveB();
                                    }).start())
                    ),
                    new WaitCommand(500),
                    new InstantCommand(clawServos::setBClawPos)
            );
        }
    }
}
