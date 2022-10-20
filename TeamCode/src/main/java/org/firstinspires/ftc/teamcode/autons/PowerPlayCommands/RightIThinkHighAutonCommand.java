package org.firstinspires.ftc.teamcode.autons.PowerPlayCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PickCone2Command;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PickCone3Command;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PickCone4Command;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PickCone5Command;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideHighBackCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightIThinkHighAutonCommand extends SequentialCommandGroup{
    public RightIThinkHighAutonCommand(Drivetrain drivetrain, Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                //Couter Clockwise Angles
                new DriveForwardCommand(drivetrain, 100),
                new TurnToCommand(drivetrain, 50),
//                new SlideHighFrontCommand(slide, clawMotors, clawServos),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, -40),
//                new PickCone5Command(slide, clawServos, clawMotors),


                new DriveForwardCommand(drivetrain, 40),
                new TurnToCommand(drivetrain, 50),
//                new SlideHighBackCommand(slide, clawMotors, clawServos),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, -40),
//                new PickCone4Command(slide, clawServos, clawMotors),


                new DriveForwardCommand(drivetrain, 40),
                new TurnToCommand(drivetrain, 50),
//                new SlideHighBackCommand(slide, clawMotors, clawServos),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, -40),
//                new PickCone3Command(slide, clawServos, clawMotors),

                new DriveForwardCommand(drivetrain, 40),
                new TurnToCommand(drivetrain, 50),
//                new SlideHighBackCommand(slide, clawMotors, clawServos),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, -40),
//                new PickCone2Command(slide, clawServos, clawMotors)
                new TurnToCommand(drivetrain, 90)
        );
    }
}