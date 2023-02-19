package org.firstinspires.ftc.teamcode.commands.Slide.SlideBCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class SlideHighBCommand extends SequentialCommandGroup {
    public SlideHighBCommand(Slide slide, Arm arm, Claw claw, TurnServo turnServo, boolean auto){
        if (auto){
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(() ->
                                    new Thread(() -> {
                                        claw.clawClose();
                                        slide.slideHigh();
                                        arm.moveHighBAuto();
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
                                        slide.slideHigh();
                                        arm.moveHighB();
                                    }).start())
                    ),
                    new WaitCommand(800),
                    new InstantCommand(turnServo::setBClawPos)
            );
        }
    }
}
