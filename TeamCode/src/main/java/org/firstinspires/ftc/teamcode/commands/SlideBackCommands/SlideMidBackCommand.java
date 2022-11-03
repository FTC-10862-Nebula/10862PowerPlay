package org.firstinspires.ftc.teamcode.commands.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideMidBackCommand extends SequentialCommandGroup {
    public SlideMidBackCommand(Slide slide, ClawMotors clawMotors, ClawServos clawServos) {
        addCommands(
                new InstantCommand(clawServos::setBClawPos),
                new InstantCommand(slide::slideMid, slide),
                new WaitCommand(150),
                new InstantCommand(clawMotors::moveMidB, clawMotors)
        );
    }
}