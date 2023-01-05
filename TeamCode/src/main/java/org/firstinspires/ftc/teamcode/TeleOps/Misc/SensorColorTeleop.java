package org.firstinspires.ftc.teamcode.TeleOps.Misc;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.GamepadTrigger;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.SensorCommands.RedIntakeCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
@Disabled
@Config
@TeleOp
public class SensorColorTeleop extends MatchOpMode {

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx armMotor;
    private ServoEx clawS1, clawS3;
        private ServoEx clawS2;
//    private CRServo clawS2;
    private MotorEx leftFront, leftRear, rightRear, rightFront;
    private MotorEx liftMotor1, liftMotor2;
    private ColorSensor colorSensor;

//    private Encoder leftEncoder, rightEncoder, frontEncoder;

    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;


    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private MecanumDrive drivetrain;
    private Slide slide;
    private SensorColor sensorColor;
//    private StandardTrackingWheelLocalizer standardTrackingWheelLocalizer;
    //    private TagVision vision;

//    private Button one;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        arm = new Arm(telemetry, hardwareMap);
        clawServos = new ClawServos( telemetry, hardwareMap);
        drivetrain = new MecanumDrive(hardwareMap, telemetry, true);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
        sensorColor = new SensorColor(colorSensor, hardwareMap, telemetry);
//        clawServos.setDefaultCommand(new RedIntakeCommand(drivetrain, slide, clawServos, arm, sensorColor));
//        vision = new TagVision(hardwareMap, "Webcam 1", telemetry);

        sensorColor.setDefaultCommand(new RedIntakeCommand(drivetrain, slide, clawServos, arm, sensorColor));

        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false, 2));
    }


    @Override
    public void configureButtons() {
        Button intakeD1Trigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
//                    .whenPressed(new SlideLowBCommand(slide, arm, clawServos))
                .whenPressed(new InstantCommand(clawServos::clawOpen));
        Button outtakeD1Trigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                .whenPressed(new InstantCommand(clawServos::clawClose));
    }

    @Override
    public void matchLoop() {
    }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){
//        sensorColor.periodic();
//        telemetry.update();
    }
}
