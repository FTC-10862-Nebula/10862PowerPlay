package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.driveCommands.teleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.driveCommands.teleopCommands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.intakeAndOutake.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.intakeAndOutake.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.slide.SlideMoveManual;
import org.firstinspires.ftc.teamcode.commands.slide.slideBCommands.SlideIntakeBCommandT;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideGroundFCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideHighFCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideLowFCommand;
import org.firstinspires.ftc.teamcode.commands.slide.slideFCommands.SlideMidFCommand;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Config
@TeleOp
public class TeleOpMain extends MatchOpMode {

    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;


    // Subsystems
    private Arm arm;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
//    private SensorColor sensorColor;
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
    }


    @Override
    public void configureButtons() {
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false));

        slide.setDefaultCommand(new SlideMoveManual(slide, operatorGamepad::getRightY));
//        arm.setDefaultCommand();

        //Drive Stuff - D1
        Button recenterIMUButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
                .whenPressed(new InstantCommand(drivetrain::reInitializeIMU));

        //Slowmode - D1
        Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER))
                .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));

        /****************/



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
                .whenPressed(new SlideIntakeBCommandT(slide, arm, claw, turnServo)));
        Button resetArmUp = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(arm::moveReset));

        //Slide Manual - D2
        Button slideUpButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT))
                .whileHeld(slide::upSlideManual)
                .whenReleased(slide::stopSlide);
        Button slideDownButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT))
                .whileHeld(slide::downSlideManual)
                .whenReleased(slide::stopSlide);

        //Arm Manual - D2
        Button armRaiseButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER))
                .whileHeld(arm::raiseClawManual)
                .whenReleased(arm::stopArm);
        Button armLowerButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(arm::lowerClawManual)
                .whenReleased(arm::stopArm);

//            PIDF Controllers Resets
        Button armMotorResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.START))
                .whenPressed(arm::encoderReset);
        Button slideEncoderResetButton = (new GamepadButton(operatorGamepad, GamepadKeys.Button.BACK))
                .whenPressed(slide::encoderReset);
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
    }
}
