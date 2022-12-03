package org.firstinspires.ftc.teamcode.autons.AutonCommands.Mid;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands.SlideMidFrontCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightMidAutonCommand extends SequentialCommandGroup{
    public RightMidAutonCommand(DrivetrainCOrrect drivetrainCorrect, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                //Counter Clockwise Angles
                new DriveForwardCommand(drivetrainCorrect, 65),
                new TurnToCommand(drivetrainCorrect, 50),
                new SlideMidFrontCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrainCorrect, 95),
                new DriveForwardCommand(drivetrainCorrect, -40)
        );
    }
}