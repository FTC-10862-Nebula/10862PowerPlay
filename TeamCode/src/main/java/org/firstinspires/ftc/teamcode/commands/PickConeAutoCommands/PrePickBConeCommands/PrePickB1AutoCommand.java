package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PrePickB1AutoCommand extends SequentialCommandGroup   {
    public PrePickB1AutoCommand(Slide slide, ClawServos clawServos, Arm arm){
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(clawServos::clawAutoClose),
                        new InstantCommand(arm::moveIntakeBAuto)
                ),
                new InstantCommand(clawServos::setBClawPos),
                new ParallelCommandGroup(
                        new InstantCommand(clawServos::clawOpen),
                        new InstantCommand(slide::slideCone5)
                )
        );
    }
}