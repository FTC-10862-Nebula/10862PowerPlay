package org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.AutonPathings.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideMidBackCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class LeftMidAutonCommand extends SequentialCommandGroup{
    public LeftMidAutonCommand(Drivetrain drivetrain, Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, 70),
                new TurnToCommand(drivetrain, 135),
                new SlideMidBackCommand(slide, clawMotors, clawServos),
                new TurnToCommand(drivetrain, 90)

        );
    }
}