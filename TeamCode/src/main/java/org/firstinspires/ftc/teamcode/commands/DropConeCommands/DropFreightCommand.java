package org.firstinspires.ftc.teamcode.commands.DropConeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DropFreightCommand extends SequentialCommandGroup {

    public DropFreightCommand(Drivetrain drivetrain){
        addRequirements(drivetrain);
        addCommands(
                //Code
        );
    }

}
