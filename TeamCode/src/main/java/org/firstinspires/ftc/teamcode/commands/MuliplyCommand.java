package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

//TODO: TEST!!!
public class MuliplyCommand extends SequentialCommandGroup{
    public MuliplyCommand( int multiply, Command... command){
        for(int num = 0; num != multiply; num++)
        {
            addCommands(
                    command
            );
        };

    }
}