package org.firstinspires.ftc.teamcode.commands.AutoCommands.AutoConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickConeCommand extends SequentialCommandGroup  {
    public PickConeCommand(Slide slide, Claw claw){
        addCommands(
                new InstantCommand(claw::clawClose),
//                new InstantCommand(claw::clawAutoClose),
                new WaitCommand(1000),
                new InstantCommand(slide:: slideLow)
        );
    }
}