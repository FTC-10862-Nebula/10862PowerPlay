package org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.PrePickConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PrePickC3FCommand extends SequentialCommandGroup{
    public PrePickC3FCommand(Slide slide, ClawServos clawServos, Arm arm){
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(arm::moveIntakeFAuto),
                new InstantCommand(clawServos::setFClawPos),
                new InstantCommand(clawServos::clawOpen),
                new InstantCommand(slide::slideCone3)

        );
    }
}