package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class SlideLowBCommand extends SequentialCommandGroup {
    public SlideLowBCommand(Slide slide, Arm arm, Claw claw, TurnServo turnServo, boolean auto) {
        if (auto){
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
                                        slide.slideLow();
                                        arm.moveBAuto();
                                    }).start())
                    ),
                    new WaitCommand(200),
                    new InstantCommand(turnServo::setBClawPos)
            );
        }
        else {
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
                                        slide.slideLow();
                                        arm.moveB();
                                    }).start())
                    ),
                    new WaitCommand(800),
                    new InstantCommand(turnServo::setBClawPos)
            );
        }
    }
}
