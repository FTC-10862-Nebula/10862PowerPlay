package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideGroundBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//@Disabled
@Autonomous
public class TrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TrajectoryVelocityConstraint vel = MecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accel = MecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, telemetry, false);
        drivetrain.init();
        ClawServos clawServos = new ClawServos(telemetry, hardwareMap);
        Arm arm = new Arm(telemetry, hardwareMap);
        Slide slide = new Slide(telemetry, hardwareMap);

        clawServos.clawClose();
        Pose2d startPose = new Pose2d(0, 0,0);


        TrajectorySequence preLoad = drivetrain.trajectorySequenceBuilder(startPose)
                .strafeRight(50)
                .strafeRight(21)
//                .lineToSplineHeading(new Pose2d(70, 5))
//                .addDisplacementMarker(new SlideGroundBCommand(slide, arm, clawServos, true))
                .addTemporalMarker(() -> new SlideMidFCommand(slide, arm, clawServos, true))
                .build();

        TrajectorySequence cycle1Pickup = drivetrain.trajectorySequenceBuilder(preLoad.end())
                .forward(5, vel, accel)
                .strafeRight(5)
                .addDisplacementMarker(clawServos::clawOpen)
                .addTemporalMarker(() -> {
                    slide.slideMid();
                    new InstantCommand(slide::dropSlide);
                    new SlideMidFCommand(slide, arm, clawServos, true);
                })
                .build();
        drivetrain.setPoseEstimate(startPose);


        waitForStart();

        drivetrain.followTrajectorySequenceAsync(preLoad);
        new SlideGroundBCommand(slide, arm, clawServos, true);

        if (isStopRequested()) return;

        }

    }
