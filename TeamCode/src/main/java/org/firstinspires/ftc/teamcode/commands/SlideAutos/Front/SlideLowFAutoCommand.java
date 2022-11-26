package org.firstinspires.ftc.teamcode.commands.SlideAutos.Front;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideLowFAutoCommand extends SequentialCommandGroup {
    public SlideLowFAutoCommand(Slide slide, Arm arm, ClawServos clawServos) {
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(slide::slideLow, slide),
                new InstantCommand(arm::moveFAuto, arm),
                new WaitCommand(850),
                new InstantCommand(clawServos::setFClawPos)
        );
    }
}
