package org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.AutonPathings.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideHighFrontCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class LeftHighAutonCommand extends SequentialCommandGroup{
    public LeftHighAutonCommand(Drivetrain drivetrain, Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, 130),
                new TurnToCommand(drivetrain, 135),
                new SlideHighFrontCommand(slide, clawMotors, clawServos, clawMotors.getFlip()),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, 30),
//                new PickCone5Command(slide, clawServos, clawMotors),

                new DriveForwardCommand(drivetrain, -30),
                new TurnToCommand(drivetrain, 135),
//                new SlideHighFrontCommand(slide, clawMotors, clawServos, clawMotors.getFlip()),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, 30),
//                new PickCone4Command(slide, clawServos, clawMotors),

                new DriveForwardCommand(drivetrain, -30),
                new TurnToCommand(drivetrain, 135),
//                new SlideHighFrontCommand(slide, clawMotors, clawServos, clawMotors.getFlip()),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, 30),
//                new PickCone3Command(slide, clawServos, clawMotors),

                new DriveForwardCommand(drivetrain, -30),
                new TurnToCommand(drivetrain, 135),
//                new SlideHighFrontCommand(slide, clawMotors, clawServos, clawMotors.getFlip()),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, 30)
//                new PickCone2Command(slide, clawServos, clawMotors)
        );
    }
}