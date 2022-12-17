package org.firstinspires.ftc.teamcode.autons.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Autonomous(name = "FaithCam", group = "RED/BLUE")
public class FaithAuton extends MatchOpMode {
//    private ATDetector tagDetector;

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    // Gamepad
//    private GamepadEx driverGamepad;

    Drivetrain drivetrain;
    Slide slide;
    Arm arm;
    ClawServos clawServos;
    @Override
    public void robotInit() {
         drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));

        slide= new Slide(telemetry,hardwareMap);
         arm = new Arm(telemetry,hardwareMap);
         clawServos = new ClawServos(telemetry,hardwareMap);
        while (!isStarted() && !isStopRequested()){
            waitForStart();
        }
    }

    public void matchStart() {
//        Drivetrain drivetrain;
        schedule(
                new SequentialCommandGroup(
                        new StrafeRightCommand( drivetrain, 55),
                        new SlideHighBCommand(slide,arm, clawServos, true),
                        new DropAutoConeCommand(clawServos, slide, arm,true),
                        new SlideResetUpAutonCommand(slide, arm, clawServos)
//                        new
        )
        );

    }
};