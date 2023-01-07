package org.firstinspires.ftc.teamcode.commands.ExampleCommand;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class ParallelExCommand extends ParallelCommandGroup{
    public ParallelExCommand(Drivetrain drivetrain){
        addRequirements(drivetrain);    //Add Subsystems that you need to run this Command
        addCommands(
                //Commands that will run automatically - Each subsystem can only be used once
                //Can also be implemented like below
                new ParallelCommandGroup(
                        //Commands
                )

        );
    }
}