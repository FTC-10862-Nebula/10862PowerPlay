package org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawServos;

public class DropConeCommand extends SequentialCommandGroup {

    public DropConeCommand(ClawServos clawServos){
        addCommands(
                new InstantCommand(clawServos::clawOpen),
                new InstantCommand(clawServos::outtakeClaw),
                new WaitCommand(10),
                new InstantCommand(clawServos::stopClaw)
        );
    }

}
