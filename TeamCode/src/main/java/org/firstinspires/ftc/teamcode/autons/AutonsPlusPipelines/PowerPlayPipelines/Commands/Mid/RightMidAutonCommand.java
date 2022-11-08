package org.firstinspires.ftc.teamcode.autons.AutonsPlusPipelines.PowerPlayPipelines.Commands.Mid;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands.SlideMidFrontCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightMidAutonCommand extends SequentialCommandGroup{
    public RightMidAutonCommand(Drivetrain drivetrain, Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                //Counter Clockwise Angles
                new DriveForwardCommand(drivetrain, 65),
                new TurnToCommand(drivetrain, 50),
                new SlideMidFrontCommand(slide, clawMotors, clawServos),
                new TurnToCommand(drivetrain, 95),
                new DriveForwardCommand(drivetrain, -40)
        );
    }
}