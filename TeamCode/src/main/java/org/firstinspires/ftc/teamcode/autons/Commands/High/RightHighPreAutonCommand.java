package org.firstinspires.ftc.teamcode.autons.Commands.High;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideHighBAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighPreAutonCommand extends SequentialCommandGroup{
    public RightHighPreAutonCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, -55),
                new TurnToCommand(drivetrain, 48, true),
                new SlideHighBAutoCommand(slide, arm, clawServos),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrain ,-8.1),
                new WaitCommand(500),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain ,8),
                new TurnToCommand(drivetrain, 90),
                new SlideResetAutonFCommand(slide, arm, clawServos)
//                new InstantCommand(arm::moveReset, arm),
        );
    }
}