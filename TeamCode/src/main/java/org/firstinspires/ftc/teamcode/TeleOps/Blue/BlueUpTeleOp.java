package org.firstinspires.ftc.teamcode.TeleOps.Blue;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadTrigger;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeAndDropConeCommands.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.SensorCommands.BlueIntakeTeleopCommand;
import org.firstinspires.ftc.teamcode.commands.SensorCommands.RedIntakeTeleopCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideBackCommands.SlideResetBCommandT;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideGroundFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideLowFCommand;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Config
@TeleOp(name = "BLUE Up")
public class BlueUpTeleOp extends MatchOpMode {
    int choice = 2;


//    private Encoder leftEncoder, rightEncoder, frontEncoder;

    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;


    // Subsystems
    private Arm arm;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private SensorColor sensorColor;
    private TurnServo turnServo;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        arm = new Arm(telemetry, hardwareMap);
        claw = new Claw(telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, true), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);

        sensorColor = new SensorColor(hardwareMap, telemetry);
//        claw.setDefaultCommand(new BlueIntakeTeleopCommand(slide, claw, sensorColor));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false, choice));
    }


    @Override
    public void configureButtons() {
        //Drive Stuff - D1
            (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
//                    .whenPressed(new DefaultDriveCommand(drivetrain, driverGamepad, true, choice));
                    .whenPressed( new InstantCommand(drivetrain::reInitializeIMU));
            (new GamepadButton(driverGamepad, GamepadKeys.Button.BACK))
                    .whenPressed(new DefaultDriveCommand(drivetrain, driverGamepad,false, choice));

        //Slowmode - D1
            Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
                    .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad, choice));

        //Claw Servo Intake/Outtake - D1
            /*Button intakeD1Trigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                .whenPressed(new PickConeCommand(claw, slide, arm));
            Button outtakeD1Trigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                .whenPressed(new DropConeCommand(claw, slide, arm, drivetrain));*/
            Button intakeD2Trigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                .whenPressed(new PickConeCommand(claw, slide, arm));
            Button outtakeD2Trigger = (new GamepadTrigger(operatorGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                .whenPressed(new DropConeCommand(claw, slide, arm));

        //Slide positions - D2
            Button groundBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                    .whenPressed(new SlideGroundFCommand(slide, arm, claw, turnServo, false)));
            Button lowBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new SlideLowFCommand(slide, arm, claw, turnServo, false)));
            Button midBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new SlideMidFCommand(slide, arm, claw, turnServo, false)));
            Button highBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(new SlideHighFCommand(slide, arm, claw, turnServo, false)));

            Button resetBSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(new SlideResetBCommandT(slide, arm, claw, turnServo)));
            Button resetArmUp = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                    .whenPressed(arm::moveReset));

        //Slide Manual - D2
            Button slideUpButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP))
                    .whileHeld(slide::upSlideManual);
            Button slideDownButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN))
                    .whileHeld(slide::downSlideManual);

        //Arm Manual - D2
            Button armRaiseButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER))
                    .whileHeld(arm::raiseClawManual);
    //                .whenPressed(arm::raiseClawManual)
    //                .whenReleased(arm::stopClaw));
            Button armLowerButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER))
                    .whileHeld(arm::lowerClawManual);
    //                .whenPressed(arm::lowerClawManual)
    //                .whenReleased(arm::stopClaw));

        //Claw Servo 3 Buttons - D1
            /*Button s3FButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y))
                    .whenPressed(claw::setFClawPos);
            Button s3BButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A))
                    .whenPressed(claw::setBClawPos);*/



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
                    .whenPressed(new SlideGroundFCommand(slide, clawMotors, claw)));
            Button lowFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                    .whenPressed(new SlideLowFCommand(slide, clawMotors, claw)));
            Button midFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                    .whenPressed(new SlideMidFCommand(slide, clawMotors, claw)));
            Button highFSlideButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                    .whenPressed(new SlideHighFCommand(slide, clawMotors, claw)));


            //Claw Servo Manual Rotation - D1
            Button plusClaw3Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP))
                    .whenPressed(claw::addClaw3Pos);
            Button subClaw3Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN))
                    .whenPressed(claw::subClaw3Pos);
            //Claw Servo Manual In/Out - D1
            Button plusClaw1Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT))
                    .whenPressed(claw::addClaw1Pos);
            Button subClaw1Button = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT))
                    .whenPressed(claw::subClaw1Pos);
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
    public void robotPeriodic(){
//        telemetry.addData("Front Encoder: ", StandardTrackingWheelLocalizer.frontEncoderPos);
//        telemetry.addData("Left Encoder: ", StandardTrackingWheelLocalizer.leftEncoderPos);
//        telemetry.addData("Right Encoder: ", StandardTrackingWheelLocalizer.rightEncoderPos);
    }
}
