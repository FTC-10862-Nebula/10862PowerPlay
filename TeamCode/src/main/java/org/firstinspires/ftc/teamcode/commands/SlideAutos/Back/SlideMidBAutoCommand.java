package org.firstinspires.ftc.teamcode.commands.SlideAutos.Back;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideMidBAutoCommand extends SequentialCommandGroup {
    public SlideMidBAutoCommand(Slide slide, Arm arm, ClawServos clawServos) {
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(slide::slideAutoMid, slide),
                new InstantCommand(arm::moveBAuto, arm),
                new WaitCommand(650),
                new InstantCommand(clawServos::setBClawPos)
        );
    }
}