package org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetAutonFCommand extends SequentialCommandGroup{
    public SlideResetAutonFCommand(Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new InstantCommand(clawServos::setFClawPos),
//                new InstantCommand(
//                        () -> new Thread(() -> {
//                            clawServos.clawClose();
//                            arm.moveIntakeF();
//                            slide.slideResting();
//                        }).start()
//                ),
//                new WaitCommand(200),
                new InstantCommand(
                        () -> new Thread(() -> {
                            arm.moveReset();
                            slide.slideResting();
                            clawServos.setFClawPos();
                        }).start()
                )
//                new InstantCommand(clawServos::clawOpen)
//                new InstantCommand(clawServos::clawClose, clawServos),
//                new WaitCommand(400),
//                new InstantCommand(arm::moveReset, arm),
//                new InstantCommand(slide::slideResting, slide),
//                new InstantCommand(clawServos::setFClawPos)
        );
    }
}
