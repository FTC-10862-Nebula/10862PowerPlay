package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PrePickC3BCommand extends SequentialCommandGroup{
    public PrePickC3BCommand(Slide slide, ClawServos clawServos, Arm arm){
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(arm::moveIntakeBAuto),
                new InstantCommand(clawServos::setBClawPos),
                new InstantCommand(clawServos::clawOpen),
                new InstantCommand(slide::slideCone3)

        );
    }
}