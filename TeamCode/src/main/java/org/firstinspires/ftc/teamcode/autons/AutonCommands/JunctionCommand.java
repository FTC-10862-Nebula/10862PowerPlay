package org.firstinspires.ftc.teamcode.autons.AutonCommands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.pipelines.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.subsystems.Misc.Vision.JunctionVision;

import java.util.HashMap;

public class JunctionCommand extends SequentialCommandGroup {
    public  JunctionCommand (JunctionVision junctionVision, Drivetrain drivetrain){
        addCommands(
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(TeamMarkerPipeline.Position.LEFT, new SequentialCommandGroup(
                            //Low
                            new TurnCommand(drivetrain, -3)
//                            new SplineCommand(drivetrain, new Vector2d(21.5,25.8), Math.toRadians(47))
                            )
                    );
                    put(TeamMarkerPipeline.Position.MIDDLE, new SequentialCommandGroup(
                            //Mid
                            new TurnCommand(drivetrain, 0)
//                    new SplineCommand(drivetrain, new Vector2d(22.8,27.5), Math.toRadians(13))
                            )
                    );
                    put(TeamMarkerPipeline.Position.RIGHT, new SequentialCommandGroup(
                            //High
                            new TurnCommand(drivetrain, 3)
//                    new SplineCommand(drivetrain, new Vector2d(23.5,26), Math.toRadians(10))
                            )
                    );
                }}, junctionVision::getCurrentPosition)
        );
    }


}
