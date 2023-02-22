package org.firstinspires.ftc.teamcode.opmode.auto.misc.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.misc.StickVision;
@Disabled
@Autonomous
public class StickObserverOwnTest extends MatchOpMode {
    private Drivetrain drivetrain;
    private StickVision stickVision;

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
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