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
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideGroundBackCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideHighBackCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideLowBackCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideMidBackCommand;


import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFrontCommands.SlideResetFrontCommandT;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

@Config
@TeleOp(name = "MainTeleop")
public class MainTeleOp extends MatchOpMode {

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

        arm = new Arm(armMotor, telemetry, hardwareMap);
        clawServos = new ClawServos(clawS1, clawS2, clawS3, telemetry, hardwareMap);
        drivetrain = new Drivetrain(new SampleMecanumDrive(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(liftMotor1, liftMotor2, telemetry, hardwareMap);
//        vision = new Vision(hardwareMap, "Webcam 1", telemetry);

        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false));

//            slide.setDefaultCommand(new SlideDefaultCommand(slide, operatorGamepad));    // - works but stops pid from working
//              upSlideManual(slide, operatorGamepad,left);
//        slide.upSlideManual(operatorGamepad, slide);

    }


    @Override
    public void configureButtons() {
        //Drive Stuff - D1
            Button robotDriveButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
                    .whenPressed(new DefaultDriveCommand(drivetrain,driverGamepad,  true));
            Button fieldDriveButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.BACK))
                    .whenPressed(new DefaultDriveCommand(drivetrain,driverGamepad,false));
        //Slowmode - D1
            Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER))
                    .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));

        //Claw Servo Intake/Outtake - D1/D2
            Button intakeD1Trigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                .whenPressed(new PickConeCommand(clawServos, slide));
            Button outtakeD1Trigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                .whenPressed(new DropConeCommand(clawServos, slide, arm));
//            Button intakeD2Trigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
//                .whenPressed(new PickConeCommand(clawServos, slide));
//            Button outtakeD2Trigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
//                .whenPressed(new DropConeCommand(clawServos, slide));

        //Slide positions - D2
            Button groundBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(new SlideGroundBackCommand(slide, arm, clawServos)));
            Button lowBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new SlideLowBackCommand(slide, arm, clawServos)));
            Button midBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new SlideMidBackCommand(slide, arm, clawServos)));
            Button highBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(new SlideHighBackCommand(slide, arm, clawServos)));

            Button resetFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SlideResetFrontCommandT(slide, arm, clawServos)));

        //Slide Manual - D2
            Button slideUpTrigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                    .whenPressed(slide::upSlideManual)
                    .whenReleased(slide::stopSlide);
            Button slideDownTrigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                    .whenPressed(slide::downSlideManual)
                    .whenReleased(slide::stopSlide);

        //Arm Manual - D2
            Button armRaiseButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(arm::raiseClawManual)
                .whenReleased(arm::stopClaw));
            Button armLowerButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(arm::lowerClawManual)
                .whenReleased(arm::stopClaw));



            /*
            Button groundFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(new SlideGroundFrontCommand(slide, clawMotors, clawServos)));
            Button lowFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new SlideLowFrontCommand(slide, clawMotors, clawServos)));
            Button midFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new SlideMidFrontCommand(slide, clawMotors, clawServos)));
            Button highFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(new SlideHighFrontCommand(slide, clawMotors, clawServos)));


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
        //Claw Servo 3 Buttons - D1
            Button s3FButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y))
                    .whenPressed(clawServos::setFClawPos);
            Button s3BButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                    .whenPressed(clawServos::setBClawPos);
        //PIDF Controllers Resets
            Button clawMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                    .whenPressed(arm::encoderReset);
            Button slideEncoderResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.BACK))
                    .whenPressed(slide::encoderReset);
    }

    @Override
    public void matchLoop() {
//        slide.setPower(-operatorGamepad.getLeftY());   - no worky
//        if(-operatorGamepad.getLeftY()>=0.5 || -operatorGamepad.getLeftY()<= -0.5)
//        {
//            slide.automaticFalse();
//            slide.setPower(-operatorGamepad.getLeftY());
//            if(-operatorGamepad.getLeftY()>=-0.1&& -operatorGamepad.getLeftY()<=0.1)
//            {
//                slide.stopSlide();
//            }

//        }
//        else if(-operatorGamepad.getLeftY()==0){
//            slide.stopSlide();
//        }
        //Works but not tooo much
    }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){}
}
