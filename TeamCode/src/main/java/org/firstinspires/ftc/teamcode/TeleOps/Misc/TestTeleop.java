package org.firstinspires.ftc.teamcode.TeleOps.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultArmCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Config
@TeleOp(name = "Test")
public class TestTeleop extends MatchOpMode {
    int choice = 2;
    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;


    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private MecanumDrive drivetrain;
    private Slide slide;
//    private StandardTrackingWheelLocalizer standardTrackingWheelLocalizer;
    //    private TagVision vision;


    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        arm = new Arm(telemetry, hardwareMap);
        clawServos = new ClawServos(telemetry, hardwareMap);
        drivetrain = new MecanumDrive(hardwareMap, telemetry);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
//        vision = new TagVision(hardwareMap, "Webcam 1", telemetry);

//        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
//        drivetrain.setPoseEstimate(PoseStorage.currentPose);
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false, choice));
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
