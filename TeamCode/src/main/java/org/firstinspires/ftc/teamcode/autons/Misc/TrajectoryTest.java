package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Treads.thing.Drivethreadcomment;
import org.firstinspires.ftc.teamcode.driveTrainAuton.DriveConstants;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TrajectoryVelocityConstraint vel = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accel = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        Drivetrain normDrive = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ClawServos clawServos = new ClawServos(telemetry, hardwareMap);
        Arm arm = new Arm(telemetry, hardwareMap);
        Slide slide = new Slide(telemetry, hardwareMap);

        clawServos.clawClose();
        Pose2d startPose = new Pose2d(0, 0,0);
//        Thread t = new Thread(() -> {
//            new DriveForwardCommand(normDrive, 12);
//            new SlideHighBackCommand(slide, arm, clawServos);
//        });
//        Thread two = new Thread(() -> {
//            new StrafeRightCommand(normDrive, 5);
//            new SlideMidBackCommand(slide, arm, clawServos);
//        });

        TrajectorySequence preLoad = drive.trajectorySequenceBuilder(startPose)
                .forward(5, vel, accel)
//                .addTemporalMarker(t.start())
//                .addTemporalMarker(clawServos::clawOpen)
//                .UNSTABLE_addTemporalMarkerOffset(.5, clawServos::clawOpen)
                .build();

        TrajectorySequence cycle1Pickup = drive.trajectorySequenceBuilder(preLoad.end())
//                .forward(5, vel, accel)
                .strafeRight(5)

                .build();


        drive.setPoseEstimate(startPose);


        waitForStart();
        if (isStopRequested()) return;

//        ThreadComman.clearGroupedCommand();
//        slide.slideHigh();
        drive.followTrajectorySequence(preLoad);
        clawServos.clawOpen();

//        sleep(10);
//        t.start();
        new Drivethreadcomment(normDrive, slide, arm, clawServos);

//        two.start();

        new InstantCommand(()->drive.followTrajectorySequence(preLoad));
        drive.followTrajectorySequence(preLoad);



        }

        // Put pose in pose storage (so it can be used in teleOp)
//        PoseStorage.currentPose = drive.getPoseEstimate();
//    Test.currentAngle = drive.getAngle();
    }
