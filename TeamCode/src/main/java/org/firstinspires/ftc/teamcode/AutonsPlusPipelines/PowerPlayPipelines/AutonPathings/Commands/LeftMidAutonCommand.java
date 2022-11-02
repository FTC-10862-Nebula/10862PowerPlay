package org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlayPipelines.AutonPathings.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideMidFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideResetFrontCommandT;
import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class LeftMidAutonCommand extends SequentialCommandGroup{
    public LeftMidAutonCommand(Drivetrain drivetrain, Slide slide, ClawMotors clawMotors, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, 70),
                new TurnToCommand(drivetrain, 315, true),
                new SlideMidFrontCommand(slide, clawMotors, clawServos, clawMotors.getFlip()),
                new WaitCommand(1200),
                new DropConeCommand(clawServos, slide),
                new WaitCommand(1100),
                new SlideResetFrontCommandT(slide, clawMotors, clawServos, clawMotors.getFlip()),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, 30),
                new TurnToCommand(drivetrain, 360),
                new DriveForwardCommand(drivetrain, 30)

        );
    }
}