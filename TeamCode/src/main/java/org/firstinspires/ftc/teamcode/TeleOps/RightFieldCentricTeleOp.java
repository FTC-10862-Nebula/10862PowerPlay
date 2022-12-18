package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadTrigger;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideGroundBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideLowBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideMidBCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideResetFCommandT;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Config
@TeleOp(name = "Right")
public class RightFieldCentricTeleOp extends MatchOpMode {

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    int choice = 3;

//    private Encoder leftEncoder, rightEncoder, frontEncoder;

    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;


    // Subsystems
    private Arm arm;
    private ClawServos clawServos;
    private Drivetrain drivetrain;
    private Slide slide;
//    private StandardTrackingWheelLocalizer standardTrackingWheelLocalizer;
    //    private Vision vision;


    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        arm = new Arm(telemetry, hardwareMap);
        clawServos = new ClawServos(telemetry, hardwareMap);
//        arm = new Arm(armMotor, telemetry, hardwareMap);
//        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
//        vision = new Vision(hardwareMap, "Webcam 1", telemetry);

//        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
        drivetrain.setPoseEstimate(PoseStorage.currentPose);
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false, choice));
    }


    @Override
    public void configureButtons() {
        //Drive Stuff - D1
        Button robotDriveButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
                .whenPressed(new DefaultDriveCommand(drivetrain, driverGamepad,  true, choice));
//                    .whenPressed( new InstantCommand(drivetrain::closeImu));
        Button fieldDriveButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.BACK))
                .whenPressed(new DefaultDriveCommand(drivetrain, driverGamepad,false, choice));

        //Slowmode - D1
        Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad, choice));

        //Claw Servo Intake/Outtake - D1
            /*Button intakeD1Trigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                .whenPressed(new PickConeCommand(clawServos, slide, arm));
            Button outtakeD1Trigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                .whenPressed(new DropConeCommand(clawServos, slide, arm, drivetrain));*/
        Button intakeD2Trigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                .whenPressed(new PickConeCommand(clawServos, slide, arm));
        Button outtakeD2Trigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                .whenPressed(new DropConeCommand(clawServos, slide, arm));

        //Slide positions - D2
        Button groundBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenPressed(new SlideGroundBCommand(slide, arm, clawServos, false)));
        Button lowBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenPressed(new SlideLowBCommand(slide, arm, clawServos, false)));
        Button midBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whenPressed(new SlideMidBCommand(slide, arm, clawServos, false)));
        Button highBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(new SlideHighBCommand(slide, arm, clawServos, false)));

        Button resetFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SlideResetFCommandT(slide, arm, clawServos)));
        Button resetArmUp = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(arm::moveReset));

        //Slide Manual - D2
        Button slideUpButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP))
                .whileHeld(slide::upSlideManual);
        Button slideDownButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN))
                .whileHeld(slide::downSlideManual);

        //Arm Manual - D2
        Button armRaiseButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(arm::raiseClawManual)
                .whenReleased(arm::stopClaw));
        Button armLowerButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(arm::lowerClawManual)
                .whenReleased(arm::stopClaw));

        //Claw Servo 3 Buttons - D1
            /*Button s3FButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y))
                    .whenPressed(clawServos::setFClawPos);
            Button s3BButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                    .whenPressed(clawServos::setBClawPos);*/



//            PIDF Controllers Resets
        Button armMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                .whenPressed(arm::encoderReset);
        Button slideEncoderResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.BACK))
                .whenPressed(slide::encoderReset);

            /*Button armFButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT))
                    .whenPressed(arm::moveF);
            Button armBButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT))
                    .whenPressed(arm::moveB);*/


            /*
            Button groundFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(new SlideGroundFCommand(slide, clawMotors, clawServos)));
            Button lowFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new SlideLowFCommand(slide, clawMotors, clawServos)));
            Button midFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new SlideMidFCommand(slide, clawMotors, clawServos)));
            Button highFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(new SlideHighFCommand(slide, clawMotors, clawServos)));


            //Claw Servo Manual Rotation - D1
            Button plusClaw3Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP))
                    .whenPressed(clawServos::addClaw3Pos);
            Button subClaw3Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN))
                    .whenPressed(clawServos::subClaw3Pos);
            //Claw Servo Manual In/Out - D1
            Button plusClaw1Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT))
                    .whenPressed(clawServos::addClaw1Pos);
            Button subClaw1Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT))
                    .whenPressed(clawServos::subClaw1Pos);
            */


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
