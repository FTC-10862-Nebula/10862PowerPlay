package org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class DropAutoConeCommand extends SequentialCommandGroup {

    public DropAutoConeCommand(Claw claw, Slide slide, Arm arm, boolean auto){
//        if(auto){
            addCommands(
                    new ParallelCommandGroup(
//                            new InstantCommand(arm::dropArmAuto),
                            new InstantCommand(slide::dropSlide),
                            new InstantCommand(claw::clawOpen)
                    ),
                    new WaitCommand(300),
                    new InstantCommand(arm::moveReset)
            );
//        }else{
//            addCommands(
//                    new InstantCommand(() ->
//                            new Thread(() -> {
//                                arm.dropArmTeleop();
//                                slide.dropSlide();
//                                claw.clawOpen();
//                            }).start()),
//                    new WaitCommand(300),
//                    new InstantCommand(arm::moveReset)
//            );
//        }

    }

}
