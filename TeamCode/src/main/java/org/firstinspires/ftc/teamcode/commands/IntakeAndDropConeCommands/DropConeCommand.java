package org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class DropConeCommand extends SequentialCommandGroup {

    public DropConeCommand(ClawServos clawServos,  Slide slide){
        addCommands(
                new InstantCommand(slide::slideMid, slide),
                new InstantCommand(clawServos::clawOpen)
//                new InstantCommand(clawServos::outtakeClaw),
//                new WaitCommand(1000),
//                new InstantCommand(clawServos::stopClaw)
        );
    }

}
