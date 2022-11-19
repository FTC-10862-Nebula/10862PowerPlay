package org.firstinspires.ftc.teamcode.autons.Commands.PrePlusHigh;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.Front.PickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoFCommands.PrePickConeCommands.PrePickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideHighAutonBCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideHighBAutoCommand;
import org.firstinspires.ftc.teamcode.commands.SlideAutos.SlideResetAutonFCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighAutonCommand extends SequentialCommandGroup{
    public RightHighAutonCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, -55),
                new TurnToCommand(drivetrain, 46.5, true),
                new SlideHighBAutoCommand(slide, arm, clawServos),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrain ,-8.1),
                new WaitCommand(500),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain ,8),
                new PrePickC5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 97),
                new DriveForwardCommand(drivetrain, 23),

                new PickC5FCommand(slide, clawServos, arm, drivetrain),
                new DriveForwardCommand(drivetrain, -25),
                new TurnToCommand(drivetrain, 33),
                new SlideHighAutonBCommand(slide, arm, clawServos),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain, -8.1),
                new InstantCommand(clawServos::clawOpen, clawServos),
                new WaitCommand(400),
                new DriveForwardCommand(drivetrain, 4.5),





                new SlideResetAutonFCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 90),
                new WaitCommand(200)
//                new InstantCommand(arm::moveReset, arm),
        );
    }
}