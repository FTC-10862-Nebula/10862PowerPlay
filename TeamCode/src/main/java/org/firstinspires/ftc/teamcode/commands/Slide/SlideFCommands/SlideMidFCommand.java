package org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class SlideMidFCommand extends SequentialCommandGroup {

    public SlideMidFCommand(Slide slide, Arm arm, Claw claw, TurnServo turnServo, boolean auto) {
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
                    new InstantCommand(turnServo::setFClawPos),
                    new WaitCommand(100)
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
                    new InstantCommand(turnServo::setFClawPos)
            );
        }

    }
}