package org.firstinspires.ftc.teamcode.commands.intakeAndOutake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class PickAutoConeCommand extends SequentialCommandGroup {

    public PickAutoConeCommand(Claw claw, Slide slide, Arm arm){
        addCommands(
                new InstantCommand(claw::clawClose, claw),
                new WaitCommand(800),
                new InstantCommand(() ->
                        new Thread(() -> {
                            slide.slidePickUp();
                            arm.moveReset();
                        }).start())
        );
    }

}
