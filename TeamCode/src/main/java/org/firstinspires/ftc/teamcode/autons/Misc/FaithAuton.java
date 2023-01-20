package org.firstinspires.ftc.teamcode.autons.Misc;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.AutoCommands.StrafeRightCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropAutoConeCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideResetUpAutonCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
@Disabled
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
    Claw claw;
    @Override
    public void robotInit() {
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();

        slide= new Slide(telemetry,hardwareMap);
        arm = new Arm(telemetry,hardwareMap);
        claw = new Claw(telemetry,hardwareMap);
        while (!isStarted() && !isStopRequested()){
            waitForStart();
        }
    }

    public void matchStart() {
//        Drivetrain drivetrain;
        schedule(
                new SequentialCommandGroup(
                        new StrafeRightCommand( drivetrain, 55),
                        new SlideHighBCommand(slide,arm, claw, true),
                        new DropAutoConeCommand(claw, slide, arm,true),
                        new SlideResetUpAutonCommand(slide, arm, claw)
//                        new
        )
        );

    }
};