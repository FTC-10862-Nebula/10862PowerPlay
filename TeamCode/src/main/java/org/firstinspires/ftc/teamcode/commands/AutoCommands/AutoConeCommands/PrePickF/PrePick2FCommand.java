package org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoConeCommands.PrePickF;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class PrePick2FCommand extends ParallelCommandGroup {
    public PrePick2FCommand(Slide slide, Claw claw, Arm arm, TurnServo turnServo){
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(claw::clawClose),
//                        new InstantCommand(claw::clawAutoClose),
                        new InstantCommand(arm::moveIntakeFAuto),
                        new InstantCommand(slide::slideCone2)
                ),
                new InstantCommand(turnServo::setFClawPos),
                new WaitCommand(150),
                new InstantCommand(claw::clawOpen)
        );
    }
}