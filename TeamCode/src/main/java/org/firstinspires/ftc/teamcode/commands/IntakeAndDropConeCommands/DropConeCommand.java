package org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class DropConeCommand extends SequentialCommandGroup {

    public DropConeCommand(ClawServos clawServos, Slide slide, ClawMotors clawMotors){
        addCommands(
                new InstantCommand(clawMotors::dropArm, clawMotors),
                new WaitCommand(100),
                new InstantCommand(clawServos::clawOpen, clawServos)
//                new InstantCommand(clawServos::outtakeClaw),
//                new WaitCommand(1000),
//                new InstantCommand(clawServos::stopClaw)
        );
    }

}
