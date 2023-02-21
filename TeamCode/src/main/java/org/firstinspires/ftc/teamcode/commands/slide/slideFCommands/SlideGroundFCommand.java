package org.firstinspires.ftc.teamcode.commands.slide.slideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class SlideGroundFCommand extends SequentialCommandGroup {
    public SlideGroundFCommand(Slide slide, Arm arm, Claw claw, TurnServo turnServo, boolean auto) {
        if (auto){
            addCommands(
//                    new InstantCommand(claw::setFClawPos),
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        turnServo.setFClawPos();
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
                                        turnServo.setFClawPos();
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