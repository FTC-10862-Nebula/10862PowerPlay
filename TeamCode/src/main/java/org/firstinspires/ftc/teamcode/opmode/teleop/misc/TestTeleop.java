package org.firstinspires.ftc.teamcode.opmode.teleop.misc;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultArmCommand;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
@Disabled
@Config
@TeleOp(name = "Test")
public class TestTeleop extends MatchOpMode {
    int choice = 2;
    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;


    // Subsystems
    private Pivot pivot;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
//    private StandardTrackingWheelLocalizer standardTrackingWheelLocalizer;
    //    private TagVision vision;


    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        pivot = new Pivot(telemetry, hardwareMap);
        claw = new Claw(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
//        vision = new TagVision(hardwareMap, "Webcam 1", telemetry);

//        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
//        drivetrain.setPoseEstimate(PoseStorage.currentPose);
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false));
        pivot.setDefaultCommand(new DefaultArmCommand(pivot, operatorGamepad));
    }


    @Override
    public void configureButtons() {

    }

    @Override
    public void matchLoop() {

    }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){ }
}
