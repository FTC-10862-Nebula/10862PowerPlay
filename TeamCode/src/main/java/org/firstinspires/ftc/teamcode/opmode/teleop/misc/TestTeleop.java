package org.firstinspires.ftc.teamcode.opmode.teleop.misc;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.driveCommands.teleopCommands.DefaultArmCommand;
import org.firstinspires.ftc.teamcode.commands.driveCommands.teleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
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
    private Arm arm;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
//    private StandardTrackingWheelLocalizer standardTrackingWheelLocalizer;
    //    private TagVision vision;


    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        arm = new Arm(telemetry, hardwareMap);
        claw = new Claw(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, false), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
//        vision = new TagVision(hardwareMap, "Webcam 1", telemetry);

//        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
//        drivetrain.setPoseEstimate(PoseStorage.currentPose);
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false));
        arm.setDefaultCommand(new DefaultArmCommand(arm, operatorGamepad));
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
