package org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.AutonPathings.Commands.High;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighAutonCommand extends SequentialCommandGroup{
    public RightHighAutonCommand(Drivetrain drivetrain, Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                //Counter Clockwise Angles
                new DriveForwardCommand(drivetrain, 130),
                new TurnToCommand(drivetrain, 50),
//                new SlideHighFrontCommand(slide, clawMotors),
                new TurnToCommand(drivetrain, 95),
                new DriveForwardCommand(drivetrain, -40),
//                new PickCone5Command(slide, clawServos, clawMotors),


                new DriveForwardCommand(drivetrain, 40),
                new TurnToCommand(drivetrain, 50),
//                new SlideHighBackCommand(slide, clawMotors),
                new TurnToCommand(drivetrain, 95),
                new DriveForwardCommand(drivetrain, -40),
//                new PickCone4Command(slide, clawServos, clawMotors),


                new DriveForwardCommand(drivetrain, 40),
                new TurnToCommand(drivetrain, 50),
//                new SlideHighBackCommand(slide, clawMotors),
                new TurnToCommand(drivetrain, 95),
                new DriveForwardCommand(drivetrain, -40),
//                new PickCone3Command(slide, clawServos, clawMotors),

                new DriveForwardCommand(drivetrain, 40),
                new TurnToCommand(drivetrain, 50),
//                new SlideHighBackCommand(slide, clawMotors),
                new TurnToCommand(drivetrain, 95),
                new DriveForwardCommand(drivetrain, -40),
//                new PickCone2Command(slide, clawServos, clawMotors)
                new TurnToCommand(drivetrain, 90)
        );
    }
}