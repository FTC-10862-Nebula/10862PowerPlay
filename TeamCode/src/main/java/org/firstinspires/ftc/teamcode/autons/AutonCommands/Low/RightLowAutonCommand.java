package org.firstinspires.ftc.teamcode.autons.AutonCommands.Low;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Front.PickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands.SlideMidFrontCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightLowAutonCommand extends SequentialCommandGroup{
    public RightLowAutonCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                //Counter Clockwise Angles
                new DriveForwardCommand(drivetrain, 65),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, 40),
                new PickC5FCommand(slide, clawServos, arm, drivetrain),

                new SlideMidFrontCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 95),
                new DriveForwardCommand(drivetrain, -40)
        );
    }
}