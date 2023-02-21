package org.firstinspires.ftc.teamcode.commands.intakeAndOutake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class DropAutoConeCommand extends SequentialCommandGroup {

    public DropAutoConeCommand(Claw claw, Slide slide, Arm arm, boolean auto){
//        if(auto){
            addCommands(
                    new InstantCommand(arm::dropArmAuto),
                    new WaitCommand(10),
                    new InstantCommand(claw::clawOpen),
                    new WaitCommand(20),
                    new InstantCommand(slide::dropSlide),
                    new WaitCommand(50)
                    );
    }

}
