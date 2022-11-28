package org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PrePickC5BCommand extends SequentialCommandGroup{
    public PrePickC5BCommand(Slide slide, ClawServos clawServos, Arm arm){
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(arm::moveIntakeBAuto),
                new InstantCommand(clawServos::setBClawPos),
                new InstantCommand(clawServos::clawOpen),
                new InstantCommand(slide::slideCone5)

        );

//        try{
////            addCommands(
//            new Thread(
//                    ()-> { slide.slideMid();
//                            clawServos.clawClose();
//                            arm.moveIntakeF();
//                    }
//                    );
//
////            addCommands(
////            new InstantCommand(slide::slideMid),
////            new InstantCommand(clawServos::clawClose),
////            new InstantCommand(arm::moveIntakeF)
////);
//        }catch (Exception e){
////            addCommands(
////                    new InstantCommand(slide::slideMid),
////                    new InstantCommand(clawServos::clawClose),
////                    new InstantCommand(arm::moveIntakeF)
////            );
//        };
//
//
//
//
////        new Thread(
////                () -> {
////                    slide.slideMid();
////                    arm.moveIntakeFAuto();
//////                    new WaitCommand(12);
////                    clawServos.clawOpen();
////
////                }
////        ).start();
    }
}