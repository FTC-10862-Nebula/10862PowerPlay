package org.firstinspires.ftc.teamcode.commands.ExampleCommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.pipelines.TeamMarkerPipeline;
import org.firstinspires.ftc.teamcode.subsystems.Misc.SingleServo;
import org.firstinspires.ftc.teamcode.subsystems.Misc.Vision.JunctionVision;

import java.util.HashMap;

public class JunctionCommand extends SequentialCommandGroup{
    public JunctionCommand(SingleServo singleServo, JunctionVision junctionVision){
        addRequirements(singleServo, junctionVision);    //Add Subsystems that you need to run this Command
        addCommands(
//                new InstantCommand(junctionVision::periodic),
        new SelectCommand(new HashMap<Object, Command>() {
            {
            put(TeamMarkerPipeline.Position.LEFT, new SequentialCommandGroup(
                    new InstantCommand(()->{singleServo.set(0.0);})
            ));
            put(TeamMarkerPipeline.Position.MIDDLE, new SequentialCommandGroup());
            put(TeamMarkerPipeline.Position.RIGHT, new SequentialCommandGroup(
                    new InstantCommand(()->{singleServo.set(1);})
            ));
            }
        }, junctionVision::getCurrentPosition)
        );
    }
}