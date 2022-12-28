package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.ExampleCommand.JunctionCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.SingleServo;
import org.firstinspires.ftc.teamcode.subsystems.Vision.JunctionVision;
import org.firstinspires.ftc.teamcode.subsystems.Vision.StickVision;
import org.firstinspires.ftc.teamcode.subsystems.Vision.TagVision;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
public class StickObserverOwnTest extends MatchOpMode {
    //Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
//    private SingleServo singleServo;
    private Drivetrain drivetrain;
//    private TagVision tagVision;
//    private JunctionVision junctionVision;
private StickVision stickVision;

    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
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