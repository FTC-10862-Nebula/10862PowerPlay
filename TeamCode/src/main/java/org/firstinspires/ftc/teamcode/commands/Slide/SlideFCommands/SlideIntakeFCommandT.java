package org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

public class SlideIntakeFCommandT extends SequentialCommandGroup {
    public SlideIntakeFCommandT(Slide slide, Arm arm, Claw claw, TurnServo turnServo){
        addRequirements(arm);
        addCommands(
                new InstantCommand(turnServo::setFClawPos),
                new ParallelCommandGroup(
                        new InstantCommand(
                            () -> new Thread(() -> {
                                claw.clawClose();
                                arm.moveIntakeF();
                                slide.slideResting();
                        }).start()
                    )
                ),
                new WaitCommand(800),
                new InstantCommand(claw::clawOpen)
        );
    }
}
