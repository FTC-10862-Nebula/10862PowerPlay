package org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideMidFCommand extends SequentialCommandGroup {
    public SlideMidFCommand(Slide slide, Arm arm, ClawServos clawServos, boolean auto) {
        if (auto){
            addCommands(
                    new InstantCommand(() ->
                            new Thread(() -> {
                                clawServos.clawClose();
                                slide.slideMid();
                                arm.moveFAuto();
                            }).start()),

                    new WaitCommand(200),
                    new InstantCommand(clawServos::setFClawPos)
            );
        }
        else {
            addCommands(
                    new InstantCommand(() ->
                            new Thread(() -> {
                                clawServos.clawClose();
                                slide.slideMid();
                                arm.moveF();
                            }).start()),

                    new WaitCommand(200),
                    new InstantCommand(clawServos::setFClawPos)
            );
        }

    }
}