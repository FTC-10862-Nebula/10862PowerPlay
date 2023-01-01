package org.firstinspires.ftc.teamcode.autons.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ExampleCommand.JunctionCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.SingleServo;
import org.firstinspires.ftc.teamcode.subsystems.Vision.JunctionVision;
import org.firstinspires.ftc.teamcode.subsystems.Vision.TagVision;

@Autonomous
public class OWNJunctionAutonTest extends MatchOpMode {
//    private ATDetector tagDetector;

    private static final double startPoseX = 0;
    private static final double startPoseY = 0;
    private static final double startPoseHeading = 0;
    private int tagNum = 0;

    //Gamepad
//    private GamepadEx driverGamepad;

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