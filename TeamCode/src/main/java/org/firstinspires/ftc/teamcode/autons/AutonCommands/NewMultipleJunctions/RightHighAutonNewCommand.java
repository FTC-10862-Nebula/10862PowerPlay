package org.firstinspires.ftc.teamcode.autons.AutonCommands.NewMultipleJunctions;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Disabled
public class RightHighAutonNewCommand extends SequentialCommandGroup{
    public RightHighAutonNewCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos){
//??DONT USE THIS SONG-----
        addCommands(
                new SplineCommand(drivetrain, new Vector2d(), 360, false)
        );
    }
}