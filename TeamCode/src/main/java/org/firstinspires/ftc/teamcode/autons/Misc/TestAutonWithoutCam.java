package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autons.Misc.Things.SplineCommandDrivetrain;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.SplinetoSplineCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.ResetPoseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickBConeCommands.PrePickB5Command;
import org.firstinspires.ftc.teamcode.commands.PickConeAutoCommands.PrePickFConeCommands.PrePick5FCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Autonomous(group = "RED/BLUE")
public class TestAutonWithoutCam extends MatchOpMode {
//    private ATDetector tagDetector;

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    // Gamepad
//    private GamepadEx driverGamepad;

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
                        new ParallelCommandGroup(
//                                new SlideHighFCommand(slide, arm, clawServos, true),
                                new SplineCommandDrivetrain(drivetrain, new Vector2d(60, 5), Math.toRadians(28))
//                                new SplineCommand(mecanumDrive, new Vector2d(60, 5), Math.toRadians(28))
                        ),
                        new WaitCommand(1000),
//                        new DropAutoConeCommand(clawServos, slide, arm, true),
                        new ParallelCommandGroup(
//                                new PrePickB5Command(slide, clawServos, arm),
                                new SplineCommandDrivetrain(drivetrain, new Vector2d(53.8, -25), Math.toRadians(270), true)
//                                new SplineCommand(mecanumDrive, new Vector2d(53.8, -25), Math.toRadians(270), true)
                        )

//                        new PickConeCommand(clawServos, slide, arm)


//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(60, 7.5), Math.toRadians(30)),
//                                new SlideHighFCommand(slide, arm, clawServos, true)
//                        ),
//                        new DropAutoConeCommand(clawServos, slide, arm, true),
//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(54, -23), Math.toRadians(270), true),
//                                new PrePickB5Command(slide, clawServos, arm)
//                        ),
//                        new PickConeCommand(clawServos, slide, arm),
//
//
//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(60, 7.5), Math.toRadians(30)),
//                                new SlideHighFCommand(slide, arm, clawServos, true)
//                        ),
//                        new DropAutoConeCommand(clawServos, slide, arm, true),
//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(54, -23), Math.toRadians(270), true),
//                                new PrePickB5Command(slide, clawServos, arm)
//                        ),
//                        new PickConeCommand(clawServos, slide, arm),
//
//
//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(60, 7.5), Math.toRadians(30)),
//                                new SlideHighFCommand(slide, arm, clawServos, true)
//                        ),
//                        new DropAutoConeCommand(clawServos, slide, arm, true),
//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(54, -23), Math.toRadians(270), true),
//                                new PrePickB5Command(slide, clawServos, arm)
//                        ),
//                        new PickConeCommand(clawServos, slide, arm)
//
//
//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(60, 7.5), Math.toRadians(30)),
//                                new SlideHighFCommand(slide, arm, clawServos, true)
//                        ),
//                        new DropAutoConeCommand(clawServos, slide, arm, true),
//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(54, -23), Math.toRadians(270), true),
//                                new PrePickB5Command(slide, clawServos, arm)
//                        ),
//                        new PickConeCommand(clawServos, slide, arm),
//
//
//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(60, 7.5), Math.toRadians(30)),
//                                new SlideHighFCommand(slide, arm, clawServos, true)
//                        ),
//                        new DropAutoConeCommand(clawServos, slide, arm, true),
//                        new ParallelCommandGroup(
//                                new SplineCommand(mecanumDrive, new Vector2d(54, -23), Math.toRadians(270), true),
//                                new PrePickB5Command(slide, clawServos, arm)
//                        ),
//                        new PickConeCommand(clawServos, slide, arm)





//        new SplinetoSplineCommand(mecanumDrive, new Pose2d(50, 10, Math.toRadians(5)), Math.toRadians(45))
//new ResetPoseCommand(mecanumDrive, new Pose2d(50, -10, Math.toRadians(5)))
//                        new SplinetoSplineCommand()
                )
        );
//        PoseStorage.currentPose = drivetrain.getPoseEstimate();
    }


};