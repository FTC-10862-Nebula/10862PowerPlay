package org.firstinspires.ftc.teamcode.commands.ExampleCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ExampleCommand extends SequentialCommandGroup{
    public ExampleCommand(MecanumDrive drivetrain){
        addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
        addCommands(
        );
    }
}