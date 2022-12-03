package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autons.AutonCommands.NewMultipleJunctions.RightHighAutonCommandSidewaysJUnctions;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeLeftCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Disabled
@Autonomous(name = "Stuff", group = "Test")
public class New1THing extends MatchOpMode {
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
    private SampleMecanumDrive driveMec;

    @Override
    public void robotInit() {
        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        arm = new Arm(armMotor, telemetry, hardwareMap);
//        driveMec = new SampleMecanumDrive(hardwareMap);
//
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
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
        this.matchStart();
    }

    public void matchStart() {
        tagNum = vision.getTag();
//        Trajectory trajectory = new TrajectoryBuilder(drivetrain.getPoseEstimate(), false, maxVelConstraint, Trajectories.accelConstraint)
//                .splineTo(new Vector2d(5,5), 360)
//                .addDisplacementMarker(() -> runCommandGroupAsThreadNow(new SlideHighFAutoCommand(slide, arm, clawServos)))
//
//                .build();

//        SequentialCommandGroup autonGroup;
        switch (tagNum) {
            case 1: { //Left
                schedule(


                        new SequentialCommandGroup(
                                new RightHighAutonCommandSidewaysJUnctions(drivetrain, slide, arm, clawServos),
                                new StrafeLeftCommand(drivetrain, 10)
                        )
                );
                return;
            }
            case 2: { //Mid
//                runCommandGroupAsThreadNow(new DropAutoConeCommand(clawServos, slide, arm));
                schedule(
                        new SequentialCommandGroup(

                                new RightHighAutonCommandSidewaysJUnctions(drivetrain, slide, arm, clawServos),
                                new StrafeRightCommand(drivetrain, 5)
                        )
                );
                return;
            }
            default: { //Right
                schedule(
                        new SequentialCommandGroup(
                                new RightHighAutonCommandSidewaysJUnctions(drivetrain, slide, arm, clawServos),
                                new StrafeRightCommand(drivetrain, 28)
                        )
                );
                return;
            }
        }
//        runCommandGroupAsThreadNow(new SlideHighFAutoCommand(slide, arm, clawServos));  //Test
    }

//    public Trajectory one (Drivetrain drive) {
//
//        Trajectory one = drive.trajectoryBuilder(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)))
////                .setReversed(false)
//                .addDisplacementMarker(runCommandGroupAsThreadNow(new SlideHighFAutoCommand(slide, arm, clawServos)))
//                .addDisplacementMarker(runCommandGroupAsThread(new TurretArmOutQuick(arm, turret, box, turret::moveLeft)))
//                .
//                .lineToLinearHeading(new Pose2d(-14, -50, Math.toRadians(90)))
////                .runCommandGroupAsThread(new TurretArmInQuick(arm, turret, box))
//                .splineToLinearHeading(new Pose2d(17, -80, Math.toRadians(0)), Math.toRadians(0))
////                .run(intake::intake)
//                .forward(40)
////                .run()
//                .build();
//        return one;
//    }

    //Test
    public void runCommandGroupAsThreadNow(SequentialCommandGroup sequentialCommandGroup) {
        new Thread(() -> {
            if (!isStopRequested()) sequentialCommandGroup.initialize();

            while (!isStopRequested() && !sequentialCommandGroup.isFinished()) {
                sequentialCommandGroup.execute();
            }
        }).start();
    }

}
