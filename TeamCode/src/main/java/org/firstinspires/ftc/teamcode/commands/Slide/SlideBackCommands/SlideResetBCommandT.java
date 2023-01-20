package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetBCommandT extends SequentialCommandGroup {
    public SlideResetBCommandT(Slide slide, Arm arm, Claw claw){
        addCommands(
                new InstantCommand(claw::setBClawPos),
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> new Thread(() -> {
                                    claw.clawClose();
                                    arm.moveIntakeB();
                                    slide.slideResting();
                                }).start()
                        )
                ),
                new WaitCommand(800),
                new InstantCommand(claw::clawOpen)

        );
    }
}
