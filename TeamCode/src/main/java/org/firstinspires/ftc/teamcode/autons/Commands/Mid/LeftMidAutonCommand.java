package org.firstinspires.ftc.teamcode.autons.Commands.Mid;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands.SlideMidFrontCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands.SlideResetFrontCommandT;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class LeftMidAutonCommand extends SequentialCommandGroup{
    public LeftMidAutonCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, 70),
                new TurnToCommand(drivetrain, 315, true),
                new SlideMidFrontCommand(slide, arm, clawServos),
                new WaitCommand(1200),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(1100),
                new SlideResetFrontCommandT(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 90),
                new DriveForwardCommand(drivetrain, 30),
                new TurnToCommand(drivetrain, 360),
                new DriveForwardCommand(drivetrain, 30)

        );
    }
}