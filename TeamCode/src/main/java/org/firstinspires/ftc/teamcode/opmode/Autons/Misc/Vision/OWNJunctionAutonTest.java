package org.firstinspires.ftc.teamcode.opmode.Autons.Misc.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.Drive.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Misc.JunctionVision;
import org.firstinspires.ftc.teamcode.subsystems.Misc.TagVision;
@Disabled
@Autonomous
public class OWNJunctionAutonTest extends MatchOpMode {
    private int tagNum = 0;

    // Subsystems
//    private SingleServo singleServo;
    private TagVision tagVision;
    private JunctionVision junctionVision;

    @Override
    public void robotInit() {
        junctionVision = new JunctionVision(hardwareMap, telemetry);
        while (!isStarted() && !isStopRequested())
        {
            junctionVision.periodic();
//            tagVision.updateTagOfInterest();
//            tagVision.tagToTelemetry();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
//        tagNum = tagVision.getTag();
//        junctionVision = new JunctionVision(hardwareMap, telemetry);
        schedule();

    }
}