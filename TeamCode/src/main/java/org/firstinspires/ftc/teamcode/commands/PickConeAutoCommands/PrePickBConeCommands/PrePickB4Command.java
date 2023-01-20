package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PrePickB4Command extends SequentialCommandGroup   {
    public PrePickB4Command(Slide slide, Claw claw, Arm arm){
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(claw::clawClose),
//                        new InstantCommand(claw::clawAutoClose),
                        new InstantCommand(arm::moveIntakeBAuto),
                        new InstantCommand(slide::slideCone4)
                ),
                new WaitCommand(150),
                new InstantCommand(claw::setBClawPos),
                new WaitCommand(150),
                new InstantCommand(claw::clawOpen)
        );
    }
}