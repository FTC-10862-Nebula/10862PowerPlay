package org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class DropAutoConeCommand extends SequentialCommandGroup {

    public DropAutoConeCommand(ClawServos clawServos, Slide slide, Arm arm){
        addRequirements(arm);
        addCommands(
                new InstantCommand(arm::dropArmAuto),
                new InstantCommand(slide::dropSlide),
                new WaitCommand(120),
                new InstantCommand(clawServos::clawOpen, clawServos),
//                new InstantCommand(clawServos::clawClose, clawServos),
//                new InstantCommand(clawServos::clawOpen, clawServos),   //Idk
                new WaitCommand(400),
                new InstantCommand(arm::moveReset)
        );
    }

}
