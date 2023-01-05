package org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickConeCommand extends SequentialCommandGroup {
    public PickConeCommand(ClawServos clawServos, Slide slide, Arm arm){
        addCommands(
                new InstantCommand(clawServos::clawClose, clawServos),
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
