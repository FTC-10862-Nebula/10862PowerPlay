package org.firstinspires.ftc.teamcode.commands.arm.frontside;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class ArmMidFrontCommand extends SequentialCommandGroup {

    public ArmMidFrontCommand(Slide slide, Pivot pivot, Claw claw, TurnServo turnServo, boolean auto) {
        if (auto){
            addCommands(
                    new WaitCommand(200),
                    new InstantCommand(claw::clawClose),
                    new ParallelCommandGroup(
                            new InstantCommand(slide::slideAutoMid),
                            new InstantCommand(pivot::moveFAuto)
                    ),
                    new WaitCommand(250),
                    new InstantCommand(turnServo::setFClawPos)
//                    new WaitCommand(100)
            );
        }
        else {
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(claw::clawClose),
                            new InstantCommand(slide::slideMid),
                            new InstantCommand(pivot::moveF)
                    ),
                    new WaitCommand(800),
                    new InstantCommand(turnServo::setFClawPos)
            );
        }

    }
}