package org.firstinspires.ftc.teamcode.autons.Misc.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.ExampleCommand.JunctionCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Misc.SingleServo;
import org.firstinspires.ftc.teamcode.subsystems.Misc.Vision.JunctionVision;
import org.firstinspires.ftc.teamcode.subsystems.Misc.Vision.TagVision;
@Disabled
@Autonomous
public class OWNJunctionAutonTest extends MatchOpMode {
    private int tagNum = 0;

    // Subsystems
    private SingleServo singleServo;
    private TagVision tagVision;
    private JunctionVision junctionVision;

    @Override
    public void robotInit() {
//        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap), telemetry, hardwareMap);
//        drivetrain.init();
//        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

//        tagVision = new TagVision(hardwareMap, "Webcam 1", telemetry);
        singleServo = new SingleServo(hardwareMap, telemetry);
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
        schedule(
                new JunctionCommand(singleServo, junctionVision)
        );

    }
}