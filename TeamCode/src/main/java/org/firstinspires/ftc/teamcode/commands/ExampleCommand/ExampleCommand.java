package org.firstinspires.ftc.teamcode.commands.ExampleCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;

//nowadays lance isnt a very common name, but in older times people were named lance a lot
public class ExampleCommand extends SequentialCommandGroup{
    public ExampleCommand(DrivetrainCOrrect drivetrainCorrect){
        addRequirements(drivetrainCorrect);    //Add Subsystems that you need to run this Command
        addCommands(
                //Commands
        );
    }
}