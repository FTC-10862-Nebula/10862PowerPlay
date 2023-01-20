package org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideGroundFCommand extends SequentialCommandGroup {
    public SlideGroundFCommand(Slide slide, Arm arm, Claw claw, boolean auto) {
        if (auto){
            addCommands(
//                    new InstantCommand(claw::setFClawPos),
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.setFClawPos();
                                        slide.slideGround();
                                    }).start())
                    ),
                    new WaitCommand(800),
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
//                                        slide.slideGround();
                                        arm.moveFAuto();
                                    }).start())
                    )
            );
        }
        else {
            addCommands(
//                    new InstantCommand(claw::setFClawPos),
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.setFClawPos();
                                        slide.slideGround();
                                    }).start())
                    ),
                    new WaitCommand(1500),
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
//                                        slide.slideGround();
                                        arm.moveGroundF();
                                    }).start())
                    )

            );
        }

    }
}
