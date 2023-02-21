package org.firstinspires.ftc.teamcode.commands.example;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ExampleCommand extends SequentialCommandGroup{
    public ExampleCommand(Drivetrain drivetrain){
        addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
        addCommands(
        );
    }
}