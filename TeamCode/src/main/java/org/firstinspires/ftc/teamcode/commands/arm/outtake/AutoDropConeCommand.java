package org.firstinspires.ftc.teamcode.commands.arm.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class AutoDropConeCommand extends SequentialCommandGroup {

    public AutoDropConeCommand(Claw claw, Slide slide, Pivot pivot, boolean auto){
//        if(auto){
            addCommands(
                    new InstantCommand(pivot::dropArmAuto),
                    new WaitCommand(40),
                    new InstantCommand(claw::clawOpen),
                    new WaitCommand(20),
                    new InstantCommand(slide::dropSlide),
                    new WaitCommand(50)
                    );
    }

}
