package org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoConeCommands.PrePickB;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class PrePickB2Command extends SequentialCommandGroup   {
    public PrePickB2Command(Slide slide, Claw claw, Arm arm, TurnServo turnServo){
        addCommands(
                new WaitCommand(67),
                new ParallelCommandGroup(
                        new InstantCommand(claw::clawClose),
//                        new InstantCommand(claw::clawAutoClose),
                        new InstantCommand(arm::moveIntakeBAuto)

                ),
                new WaitCommand(150),
                new ParallelCommandGroup(
                        new InstantCommand(slide::slideCone2),
                        new InstantCommand(turnServo::setBClawPos)
                ),
                new WaitCommand(150),
                new InstantCommand(claw::clawOpen)
        );
    }
}