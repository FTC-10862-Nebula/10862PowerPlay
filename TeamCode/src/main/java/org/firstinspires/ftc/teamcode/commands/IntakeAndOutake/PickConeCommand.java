package org.firstinspires.ftc.teamcode.commands.IntakeAndOutake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickConeCommand extends SequentialCommandGroup {
    public PickConeCommand(Claw claw, Slide slide, Arm arm){
        addCommands(
                new InstantCommand(claw::clawClose, claw),
                new WaitCommand(700),
                new InstantCommand(slide::slidePickUp)
//                new InstantCommand(() ->
//                        new Thread(() -> {
//                            slide.slidePickUp();
//                            arm.moveReset();
//                        }).start())
        );
    }
}
