package org.firstinspires.ftc.teamcode.commands.ExampleCommand;

import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.pipelines.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision.JunctionVision;

import java.util.HashMap;

public class JunctionCommand extends SequentialCommandGroup{
    public JunctionCommand(Drivetrain drivetrain, JunctionVision junctionVision){
        addRequirements(drivetrain, junctionVision);    //Add Subsystems that you need to run this Command
        addCommands(
//                new InstantCommand(junctionVision::periodic),
        new SelectCommand(new HashMap<Object, Command>() {
            {
            put(TeamMarkerPipeline.Position.LEFT, new SequentialCommandGroup(
                    new TurnCommand(drivetrain, -3)
            ));
            put(TeamMarkerPipeline.Position.MIDDLE, new SequentialCommandGroup());
            put(TeamMarkerPipeline.Position.RIGHT, new SequentialCommandGroup(
                    new TurnCommand(drivetrain, 3)
            ));
            }
        }, junctionVision::getCurrentPosition)
        );
    }
}