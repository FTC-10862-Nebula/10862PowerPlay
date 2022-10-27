package org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawServos;

public class PickConeCommand extends SequentialCommandGroup {

    public PickConeCommand(ClawServos clawServos){
        addCommands(
                new InstantCommand(clawServos::clawClose)
//                new InstantCommand(clawServos::intakeClaw),
//                new WaitCommand(4000),
//                new InstantCommand(clawServos::stopClaw)
        );
    }

}
