package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDriveCorrect;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous(name = "Stuff2", group = "Test")
public class New2THing extends LinearOpMode {
//    private ATDetector tagDetector;

    private static final double startPoseX = 0;
    private static final double startPoseY = 0;
    private static final double startPoseHeading = 0;
    private int tagNum = 0;

    //Motors and Servos
    private MotorEx armMotor;
    private ServoEx clawS1, clawS3;
    private ServoEx clawS2;
    //    private CRServo clawS2;
    private MotorEx leftFront, leftRear, rightRear, rightFront;
    private MotorEx liftMotor1, liftMotor2;

    //Gamepad
//    private GamepadEx driverGamepad;

    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;
    private Vision vision;
    private SampleMecanumDriveCorrect driveMec;

    public void runOpMode() throws InterruptedException {

        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        arm = new Arm(armMotor, telemetry, hardwareMap);
//        driveMec = new SampleMecanumDrive(hardwareMap);
//
        drivetrain = new Drivetrain(new SampleMecanumDriveCorrect(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

        vision = new Vision(hardwareMap, "Webcam 1", telemetry);
//        vision.init(hardwareMap);
        while (!isStarted() && !isStopRequested()) {
            vision.updateTagOfInterest();
            vision.tagToTelemetry();
            telemetry.update();
        }

        TrajectorySequence trajSeq = driveMec.trajectorySequenceBuilder(new Pose2d(startPoseX, startPoseY, startPoseHeading))
//                .run(clawServos::clawOpen)
                .lineToLinearHeading(new Pose2d(-14, 38, Math.toRadians(0)))
                .waitSeconds(1)
//                                .addDisplacementMarker(runCommandGroupAsThreadNow(new SlideHighFAutoCommand(slide, arm, clawServos)))
//                .run(clawServos::clawClose)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(17, 69, Math.toRadians(0)))
                .forward(30)
                .back(50)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, 38, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(17, 74, Math.toRadians(0)))
                .forward(30)
                .back(50)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, 38, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(17, 79, Math.toRadians(0)))
                .forward(30)
                .back(50)
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-14, 38, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(17, 84, Math.toRadians(0)))
                .build();

        waitForStart();

//        if (!isStopRequested())
//            drive.followTrajectorySequence(trajSeq);
    }
//    @Override
//    public void robotInit() {
//
//        this.matchStart();
//    }
//
//                                new RightHighAutonCommandSidewaysJUnctions(drivetrain, slide, arm, clawServos),
//                                new StrafeRightCommand(drivetrain, 5)
//
////        runCommandGroupAsThreadNow(new SlideHighFAutoCommand(slide, arm, clawServos));  //Test
//
//    //Test
//    public void runCommandGroupAsThreadNow(SequentialCommandGroup sequentialCommandGroup) {
//        new Thread(() -> {
//            if (!isStopRequested()) sequentialCommandGroup.initialize();
//
//            while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
//                sequentialCommandGroup.execute();
//            }
//        }).start();
//    }

}
