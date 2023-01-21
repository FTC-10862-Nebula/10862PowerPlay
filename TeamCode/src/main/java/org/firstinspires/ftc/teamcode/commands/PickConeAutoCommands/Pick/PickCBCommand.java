package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Pick;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickCBCommand extends SequentialCommandGroup  {
    public PickCBCommand(Slide slide, Claw claw){
        addCommands(
                new InstantCommand(claw::clawClose),
//                new InstantCommand(claw::clawAutoClose),
//                new WaitCommand(400),
                new WaitCommand(1000),

                new InstantCommand(slide:: slideLow)
        );
    }
}