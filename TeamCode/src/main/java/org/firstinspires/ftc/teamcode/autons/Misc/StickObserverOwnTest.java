package org.firstinspires.ftc.teamcode.autons.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Vision.StickVision;

@Autonomous
public class StickObserverOwnTest extends MatchOpMode {
    //Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
//    private SingleServo singleServo;
    private MecanumDrive drivetrain;
//    private TagVision tagVision;
//    private JunctionVision junctionVision;
private StickVision stickVision;

    @Override
    public void robotInit() {
        drivetrain = new MecanumDrive(hardwareMap, telemetry);
        drivetrain.init();
//        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

//        tagVision = new TagVision(hardwareMap, "Webcam 1", telemetry);
        stickVision = new StickVision(hardwareMap, telemetry, this);

//        singleServo = new SingleServo(hardwareMap, telemetry);
        while (!isStarted() && !isStopRequested())
        {
//            tagVision.updateTagOfInterest();
//            tagVision.tagToTelemetry();
//            stickVision.getTurning();
//            stickVision.periodic();
            telemetry.update();
        }
        this.matchStart();
    }

    public void matchStart() {
//        tagNum = tagVision.getTag();
//        junctionVision = new JunctionVision(hardwareMap, telemetry);
        schedule(
//                new TurnCommand(drivetrain, stickVision.getTurning())

        );
//        if matchLoop();        stickVision.stopCamera();

    }
}