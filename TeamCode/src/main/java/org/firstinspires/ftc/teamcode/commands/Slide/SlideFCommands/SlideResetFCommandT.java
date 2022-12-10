package org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetFCommandT extends SequentialCommandGroup {
    public SlideResetFCommandT(Slide slide, Arm arm, ClawServos clawServos){
        addRequirements(arm);
        addCommands(
                new InstantCommand(clawServos::setFClawPos),
                new InstantCommand(
                        () -> new Thread(() -> {
                            clawServos.clawClose();
                            arm.moveIntakeF();
                            slide.slideResting();
                        }).start()
                ),
                new InstantCommand(clawServos::clawOpen)

//                new WaitCommand(200),
//                new InstantCommand(clawServos::setFClawPos)
//
//                new WaitCommand(100),
//                new InstantCommand(clawServos::clawClose, clawServos),
//                new WaitCommand(200),
//                new InstantCommand(arm::moveIntakeF, arm),
//                new WaitCommand(100),
//                new InstantCommand(slide::slideResting, slide),
//                new InstantCommand(clawServos::setFClawPos),
//                new WaitCommand(200),
//                new InstantCommand(clawServos::clawOpen)
        );
    }
}
