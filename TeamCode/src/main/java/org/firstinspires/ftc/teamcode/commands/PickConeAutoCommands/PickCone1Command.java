package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickCone1Command extends SequentialCommandGroup{
    public PickCone1Command(Slide slide, ClawServos clawServos){
        addCommands(
                new InstantCommand(slide::slideCone1),
                new InstantCommand(clawServos::clawClose),
                new WaitCommand(50),
                new InstantCommand(slide:: slideLow)
        );
    }
}