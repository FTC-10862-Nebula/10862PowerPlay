package org.firstinspires.ftc.teamcode.autons.Misc;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autons.AutonCommands.LeftSpline;
import org.firstinspires.ftc.teamcode.autons.AutonCommands.RightSpline;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Autonomous(group = "RED/BLUE")
public class TestAutonWithoutCam extends MatchOpMode {

    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;
//    public MecanumDrive mecanumDrive;


    @Override
    public void robotInit() {
        clawServos = new ClawServos( telemetry, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
        slide = new Slide(telemetry, hardwareMap);
//        mecanumDrive = new MecanumDrive(hardwareMap, telemetry, false);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        //        mecanumDrive.init();
//        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
    }


    public void matchStart() {
//        waitForStart();
        schedule(
                new SequentialCommandGroup(
//                        new RightSpline(drivetrain, slide, arm, clawServos)
                      new LeftSpline(drivetrain, slide, arm, clawServos)
                )
        );
//        PoseStorage.currentPose = drivetrain.getPoseEstimate();
    }


};