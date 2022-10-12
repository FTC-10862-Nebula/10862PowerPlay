package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutonsPlusPipelines.PowerPlay.ATDetector;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideGroundBackCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideHighBackCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideLowBackCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideMidBackCommand;
import org.firstinspires.ftc.teamcode.commands.SlideBackCommands.SlideResetBackCommandT;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideGroundFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideLowFrontCommand;
import org.firstinspires.ftc.teamcode.commands.SlideFrontCommands.SlideMidFrontCommand;
import org.firstinspires.ftc.teamcode.driveTrain.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrain.SampleTankDrive;

import org.firstinspires.ftc.teamcode.subsystems.ClawMotors;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
//@Disabled
@Config
@TeleOp(name = "ClawMotorTeleop")
public class ClawMotorTeleop extends MatchOpMode {
    public static double startPoseX = 0;
    public static double startPoseY = 0;
    public static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx clawMotor;

    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;


    // Subsystems
    private ClawMotors clawMotors;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        clawMotors = new ClawMotors(clawMotor, telemetry, hardwareMap);
    }

    //Buttons
    public Button intakeF, groundF, lowF, midF, highF;
    public Button intakeB, groundB, lowB, midB, highB;

    @Override
    public void configureButtons() {
        intakeF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(clawMotors::moveClawIntakeFront));
        groundF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenPressed(clawMotors::moveClawGroundFront));
        lowF= (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)
                .whenPressed(clawMotors::moveClawLowFront));
        midF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whenPressed(clawMotors::moveClawMidFront));
        highF = (new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(clawMotors::moveClawHighFront));

        intakeB = (new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(clawMotors::moveClawIntakeBack));
        groundB = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(clawMotors::moveClawGroundBack));
        lowB= (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(clawMotors::moveClawLowBack));
        midB = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(clawMotors::moveClawMidBack));
        highB = (new GamepadButton(operatorGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(clawMotors::moveClawHighBack));
    }

    @Override
    public void matchLoop() { }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){ }
}

