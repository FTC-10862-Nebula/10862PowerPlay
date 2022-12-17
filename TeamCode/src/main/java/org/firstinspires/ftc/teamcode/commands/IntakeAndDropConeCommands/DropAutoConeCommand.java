package org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class DropAutoConeCommand extends SequentialCommandGroup {

    public DropAutoConeCommand(ClawServos clawServos, Slide slide, Arm arm, boolean auto){
//        if(auto){
            addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand(arm::dropArmAuto),
                            new InstantCommand(slide::dropSlide),
                            new InstantCommand(clawServos::clawOpen)
                    ),
//                    new InstantCommand(() ->
//                            new Thread(() -> {
//                                arm.dropArmAuto();
//                                slide.dropSlide();
//                                clawServos.clawOpen();
//                            }).start()),
                    new WaitCommand(300),
                    new InstantCommand(arm::moveReset)
            );
//        }else{
//            addCommands(
//                    new InstantCommand(() ->
//                            new Thread(() -> {
//                                arm.dropArmTeleop();
//                                slide.dropSlide();
//                                clawServos.clawOpen();
//                            }).start()),
//                    new WaitCommand(300),
//                    new InstantCommand(arm::moveReset)
//            );
//        }

    }

}
