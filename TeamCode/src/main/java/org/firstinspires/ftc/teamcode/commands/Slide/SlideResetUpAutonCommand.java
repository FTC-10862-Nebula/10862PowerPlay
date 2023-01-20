package org.firstinspires.ftc.teamcode.commands.Slide;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetUpAutonCommand extends SequentialCommandGroup{
    public SlideResetUpAutonCommand(Slide slide, Arm arm, Claw claw){
        addCommands(
                new InstantCommand(claw::clawAutoClose),
                new InstantCommand(claw::setFClawPos),
                new InstantCommand(
                        () -> new Thread(() -> {
                            arm.moveReset();
                            slide.slideResting();
                            claw.setFClawPos();
                        }).start()
                )
        );
    }
}
