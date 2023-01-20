package org.firstinspires.ftc.teamcode.commands.Slide;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class SlideResetUpAutonCommand extends SequentialCommandGroup{
    public SlideResetUpAutonCommand(Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        addCommands(
                new InstantCommand(claw::clawAutoClose),
                new InstantCommand(turnServo::setFClawPos),
                new InstantCommand(
                        () -> new Thread(() -> {
                            arm.moveReset();
                            slide.slideResting();
                            turnServo.setFClawPos();
                        }).start()
                )
        );
    }
}
