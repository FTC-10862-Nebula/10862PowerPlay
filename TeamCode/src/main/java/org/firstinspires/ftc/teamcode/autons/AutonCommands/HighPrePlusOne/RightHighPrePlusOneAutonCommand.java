package org.firstinspires.ftc.teamcode.autons.AutonCommands.HighPrePlusOne;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SlowDriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.Front.PickC5FCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePick5FCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideLowBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;


public class RightHighPrePlusOneAutonCommand extends SequentialCommandGroup{
    public RightHighPrePlusOneAutonCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
        addCommands(
                new DriveForwardCommand(drivetrain, -55),
                new TurnToCommand(drivetrain, 47, true),
                new SlideHighBCommand(slide, arm, clawServos, true),
                new WaitCommand(100),
                new SlowDriveForwardCommand(drivetrain,-8.1),
                new WaitCommand(500),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(200),

                new SlowDriveForwardCommand(drivetrain,8),
                new PrePick5FCommand(slide, clawServos, arm),
                new TurnToCommand(drivetrain, 97),
                new DriveForwardCommand(drivetrain, 25),

                new PickC5FCommand(slide, clawServos),
                new SlideLowBCommand(slide, arm, clawServos, true),
                new TurnToCommand(drivetrain, 148),
                new SlowDriveForwardCommand(drivetrain, -2),
//                new InstantCommand(clawServos::clawOpen, clawServos),
                new DropAutoConeCommand(clawServos, slide, arm),
                new WaitCommand(400),
                new SlowDriveForwardCommand(drivetrain, 3),
                new TurnToCommand(drivetrain, 90),


                new SlideResetUpAutonCommand(slide, arm, clawServos),
                new WaitCommand(200)
//                new InstantCommand(arm::moveReset, arm),
        );
    }
}