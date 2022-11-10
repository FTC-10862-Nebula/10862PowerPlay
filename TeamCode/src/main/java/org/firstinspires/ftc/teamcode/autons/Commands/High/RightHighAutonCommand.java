package org.firstinspires.ftc.teamcode.autons.Commands.High;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Back.PickC3BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Back.PickC4BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Back.PickC5BCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front.PickC3FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front.PickC4FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front.PickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.PrePickConeCOmmands.PrePickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideAutonFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBackCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideLowBackCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands.SlideHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands.SlideLowFrontCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands.SlideResetFrontCommandT;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighAutonCommand extends SequentialCommandGroup{
    public RightHighAutonCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, -75.5),
                new TurnToCommand(drivetrain, 26.3, true),
                new SlideHighBackCommand(slide, arm, clawServos),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrain ,-5),
                new WaitCommand(500),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(600),
                new DriveForwardCommand(drivetrain ,5),
                new PrePickC5FCommand(slide, clawServos, arm),
//                new SlideLowFrontCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 87),
//                new InstantCommand(clawServos::clawOpen, clawServos),
                new DriveForwardCommand(drivetrain, 41),

                new PickC5FCommand(slide, clawServos, arm, drivetrain),
                new SlideLowBackCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 150),
                new DriveForwardCommand(drivetrain, -8),
                new InstantCommand(clawServos::clawOpen, clawServos),
                new WaitCommand(400),
                new DriveForwardCommand(drivetrain, 7),
                new TurnToCommand(drivetrain, 86),

                new PickC4FCommand(slide, clawServos, arm, drivetrain),
                new SlideLowBackCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 145),
                new DriveForwardCommand(drivetrain, -8),new InstantCommand(clawServos::clawOpen, clawServos),
                new WaitCommand(400),
                new DriveForwardCommand(drivetrain, 7),
                new TurnToCommand(drivetrain, 85),

//                new PickC3FCommand(slide, clawServos, arm, drivetrain),
//                new TurnToCommand(drivetrain, 140),
//                new SlideLowBackCommand(slide, arm, clawServos),
//                new DriveForwardCommand(drivetrain, -8),new InstantCommand(clawServos::clawOpen, clawServos),
//                new WaitCommand(400),
//                new DriveForwardCommand(drivetrain, 7),
//                new TurnToCommand(drivetrain, 85),

                new SlideAutonFCommand(slide, arm, clawServos),
                new WaitCommand(200),
                new InstantCommand(arm::moveReset, arm),
                new DriveForwardCommand(drivetrain, -57),
                new TurnToCommand(drivetrain, 180)
        );
    }
}