package org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.PrePickConeCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PrePickC5FCommand extends SequentialCommandGroup{
    public PrePickC5FCommand(Slide slide, ClawServos clawServos, Arm arm){
        addCommands(
                new InstantCommand(clawServos::clawClose),
                new InstantCommand(arm::moveIntakeFAuto),
                new InstantCommand(clawServos::setFClawPos),
                new InstantCommand(clawServos::clawOpen),
                new InstantCommand(slide::slideCone5)


        );
//        new Thread(
//                () -> {
//                    slide.slideMid();
//                    arm.moveIntakeFAuto();
////                    new WaitCommand(12);
//                    clawServos.clawOpen();
//
//                }
//        ).start();
    }
}