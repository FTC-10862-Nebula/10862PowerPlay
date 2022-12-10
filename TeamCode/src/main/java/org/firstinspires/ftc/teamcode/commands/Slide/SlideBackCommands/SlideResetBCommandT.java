package org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetBCommandT extends ParallelCommandGroup {
    public SlideResetBCommandT(Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new InstantCommand(clawServos::setFClawPos),
                new InstantCommand(
                        () -> new Thread(() -> {
                            clawServos.clawClose();
                            arm.moveIntakeB();
                            slide.slideResting();
                        }).start()
                ),
                new InstantCommand(clawServos::clawOpen)

//                new InstantCommand(clawServos::clawClose, clawServos),
//                new InstantCommand(clawServos::setBClawPos),
//                new WaitCommand(800),
//                new InstantCommand(slide::slideResting, slide),
//                new InstantCommand(arm::moveIntakeB, arm),
//                new WaitCommand(600),
//                new InstantCommand(clawServos::clawOpen)
        );
    }
}
