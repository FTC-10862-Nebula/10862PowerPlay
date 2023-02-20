package org.firstinspires.ftc.teamcode.commands.Slide.SlideBCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class SlideIntakeBCommandT extends SequentialCommandGroup {
    public SlideIntakeBCommandT(Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        addCommands(
                new InstantCommand(turnServo::setBClawPos),
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> new Thread(() -> {
                                    claw.clawClose();
                                    arm.moveIntakeB();
                                }).start()
                        )
                ),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> new Thread(() -> {
                                    claw.clawOpen();
                                    slide.slideResting();

                                }).start()
                        )
                )

        );
    }
}