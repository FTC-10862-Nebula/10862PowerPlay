package org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideHighFCommand extends SequentialCommandGroup {
    public SlideHighFCommand(Slide slide, Arm arm, ClawServos clawServos, boolean auto){
        if (auto){
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        clawServos.clawClose();
                                        slide.slideAutoHigh();
                                        arm.moveHighFAuto();
                                    }).start())
                    ),
                    new WaitCommand(520),
                    new InstantCommand(clawServos::setFClawPos)
            );
        }
        else {
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        clawServos.clawClose();
                                        slide.slideHigh();
                                        arm.moveHighF();
                                    }).start())
                    ),
                    new WaitCommand(800),
                    new InstantCommand(clawServos::setFClawPos)
            );
        }
    }   
}
