package org.firstinspires.ftc.teamcode.commands.intakeAndOutake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class DropConeCommand extends SequentialCommandGroup  {

    public DropConeCommand(Claw claw, Slide slide, Arm arm){
        addCommands(
                new InstantCommand(arm::dropArmTeleop),
//                new WaitCommand(10),

//new InstantCommand(slide::dropSlide),
//                new InstantCommand(() ->
//                        new Thread(() -> {
//                            arm.dropArmTeleop();
//                            slide.dropSlide();
////                            claw.clawOpen();
//                        }).start()),
                new WaitCommand(10),
                new InstantCommand(claw::clawOpen),
                new WaitCommand(200),
                new InstantCommand(slide::dropSlide)

        );
    }

}
