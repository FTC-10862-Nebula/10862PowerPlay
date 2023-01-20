package org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideMidFCommand extends SequentialCommandGroup {

    public SlideMidFCommand(Slide slide, Arm arm, Claw claw, boolean auto) {
        if (auto){
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
                                        slide.slideAutoMid();
                                        arm.moveFAuto();
                                    }).start())
                    ),
                    new WaitCommand(200),
                    new InstantCommand(claw::setFClawPos)
            );
        }
        else {
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
                                        slide.slideMid();
                                        arm.moveF();
                                    }).start())
                    ),
                    new WaitCommand(800),
                    new InstantCommand(claw::setFClawPos)
            );
        }

    }
}