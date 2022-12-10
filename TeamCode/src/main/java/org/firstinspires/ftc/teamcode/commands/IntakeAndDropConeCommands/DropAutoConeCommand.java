package org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class DropAutoConeCommand extends ParallelCommandGroup {

    public DropAutoConeCommand(ClawServos clawServos, Slide slide, Arm arm){
        addRequirements(arm);
        addCommands(
                new InstantCommand(() ->
                        new Thread(() -> {
                            arm.dropArmAuto();
                            slide.dropSlide();
                            clawServos.clawOpen();
                        }).start()),
                new WaitCommand(300),
                new InstantCommand(arm::moveReset)
        );
    }

}
