package org.firstinspires.ftc.teamcode.commands.SlideAutos.Back;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class SlideResetAutonBCommand extends SequentialCommandGroup {
    public SlideResetAutonBCommand(Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new InstantCommand(clawServos::clawClose, clawServos),
                new WaitCommand(400),
                new InstantCommand(arm::moveReset, arm),
                new InstantCommand(slide::slideResting, slide),
                new InstantCommand(clawServos::setBClawPos)
        );
    }
}
