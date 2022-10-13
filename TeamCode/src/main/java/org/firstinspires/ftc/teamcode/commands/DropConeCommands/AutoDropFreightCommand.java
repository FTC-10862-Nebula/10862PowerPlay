package org.firstinspires.ftc.teamcode.commands.DropConeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class AutoDropFreightCommand extends SequentialCommandGroup {
    public AutoDropFreightCommand(Drivetrain drivetrain){
        addRequirements(drivetrain);
        addCommands(
                //new WaitCommand(50),
                new WaitCommand(50)
                );
    }

}
