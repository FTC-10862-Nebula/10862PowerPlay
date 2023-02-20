package org.firstinspires.ftc.teamcode.commands.Example;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.Drive.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class FaithCommand extends SequentialCommandGroup{
    public FaithCommand(Slide slide, Arm arm, Drivetrain drivetrain){
//        addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
        addCommands(
                //Commands
        new InstantCommand(arm::moveF, arm),
//        new InstantCommand(slide::slideMid),
      new InstantCommand(slide::slideMid)   ,
//          new InstantCommand(drivetrain::)

                new DriveForwardCommand(drivetrain, 30)
        );
    }
}