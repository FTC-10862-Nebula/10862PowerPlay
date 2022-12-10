package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideHighBCommand extends SequentialCommandGroup {
    public SlideHighBCommand(Slide slide, Arm arm, ClawServos clawServos, boolean auto){
        if (auto){
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        clawServos.clawClose();
                                        slide.slideHigh();
                                        arm.moveHighBAuto();
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
                                        slide.slideHigh();
                                        arm.moveHighB();
                                    }).start())
                    ),
                    new WaitCommand(200),
                    new InstantCommand(clawServos::setBClawPos)
            );
        }
    }
}
