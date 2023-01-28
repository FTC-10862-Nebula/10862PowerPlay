package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class SlideResetBCommandT extends SequentialCommandGroup {
    public SlideResetBCommandT(Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        addCommands(
                new InstantCommand(turnServo::setBClawPos),
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> new Thread(() -> {
                                    claw.clawClose();
                                    slide.slideResting();
                                }).start()
                        )
                ),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> new Thread(() -> {
                                    claw.clawOpen();
                                    arm.moveIntakeB();

                                }).start()
                        )
                )

        );
    }
}
