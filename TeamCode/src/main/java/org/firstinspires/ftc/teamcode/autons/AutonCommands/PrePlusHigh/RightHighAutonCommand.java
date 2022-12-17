package org.firstinspires.ftc.teamcode.autons.AutonCommands.PrePlusHigh;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Pick.PickCFCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePick5FCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighAutonCommand extends SequentialCommandGroup{
    public RightHighAutonCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, -55),
                new TurnToCommand(drivetrain, 46.5, true),
                new SlideHighBCommand(slide, arm, clawServos, true),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrain,-8.1),
                new WaitCommand(500),
                new DropConeCommand(clawServos, slide, arm),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain,8),
                new PrePick5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 97),
                new DriveForwardCommand(drivetrain, 23),

                new PickCFCommand(slide, clawServos),
                new DriveForwardCommand(drivetrain, -25),
                new TurnToCommand(drivetrain, 33),
                new SlideHighBCommand(slide, arm, clawServos, true),
                new WaitCommand(200),
                new SlowDriveForwardCommand(drivetrain, -8.1),
                new InstantCommand(clawServos::clawOpen, clawServos),
                new WaitCommand(400),
                new DriveForwardCommand(drivetrain, 4.5),





                new SlideResetUpAutonCommand(slide, arm, clawServos),
                new TurnToCommand(drivetrain, 90),
                new WaitCommand(200)
//                new InstantCommand(arm::moveReset, arm),
        );
    }
}