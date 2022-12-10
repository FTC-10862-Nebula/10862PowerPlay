package org.firstinspires.ftc.teamcode.commands.ExampleCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class ParallelExCommand extends ParallelCommandGroup{
    public ParallelExCommand(Drivetrain drivetrain, Slide slide, Arm arm, ClawServos clawServos, double driveDistance){
//        addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
        addCommands(
//                //Commands
                new DriveForwardCommand(drivetrain, driveDistance),
                new InstantCommand(slide::slideLow),
                new InstantCommand(arm::moveHighBAuto),
                new InstantCommand(clawServos::clawClose)
        );
    }
}