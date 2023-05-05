package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.backside.ArmIntakeBackCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmHighFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmLowFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.frontside.ArmMidFrontCommand;
import org.firstinspires.ftc.teamcode.commands.arm.intake.PickConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.outtake.DropConeCommand;
import org.firstinspires.ftc.teamcode.commands.arm.pivot.PivotMoveManual;
import org.firstinspires.ftc.teamcode.commands.arm.slide.SlideMoveManual;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.teleop.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.GamepadTrigger;
import org.firstinspires.ftc.teamcode.util.MatchOpMode;

@Config
@TeleOp
public class EmetTestTeleop extends MatchOpMode {

    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Pivot pivot;
    private Claw claw;
    private Drivetrain drivetrain;
    private Slide slide;
    private TurnServo turnServo;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);

        pivot = new Pivot(telemetry, hardwareMap);
        claw = new Claw(telemetry, hardwareMap);
        turnServo = new TurnServo(telemetry, hardwareMap);
        drivetrain = new Drivetrain(new MecanumDrive(hardwareMap, telemetry, true), telemetry, hardwareMap);
        drivetrain.init();
        slide = new Slide(telemetry, hardwareMap);
        pivot.moveInitializationPosition();
    }


    @Override
    public void configureButtons() {
        /*
         *  DRIVER
         */
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, true));

        Button recenterIMU = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y))
                .whenPressed(new InstantCommand(drivetrain::reInitializeIMU));

        Button slowMode = (new GamepadButton(driverGamepad, GamepadKeys.Button.X))
                .whileHeld(new SlowDriveCommand(drivetrain, driverGamepad, true));

//        pivot.setDefaultCommand(new PivotMoveManual(pivot, operatorGamepad::getLeftY));
        Button pivotDown = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER))
            .whileHeld(new InstantCommand(()->pivot.pivotDown()));
        Button pivotUp = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER))
            .whileHeld(new InstantCommand(()->pivot.pivotUp()));

        Button armIntake = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER))
                .whenPressed(new PickConeCommand(claw));

        Button armOuttake = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER))
                .whenPressed(new DropConeCommand(claw, slide, pivot));

        Button armLowFront = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ArmLowFrontCommand(slide, pivot, claw, turnServo, false)));

        Button armMidFront = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ArmMidFrontCommand(slide, pivot, claw, turnServo, false)));

        Button armHighFront = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ArmHighFrontCommand(slide, pivot, claw, turnServo, false)));

        Button armIntakeBack = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ArmIntakeBackCommand(slide, pivot, claw, turnServo)));

        Button pivotInitializationPosition = (new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(pivot::encoderReset));

        Button slideReset = (new GamepadButton(driverGamepad, GamepadKeys.Button.B))
                .whenPressed(slide::encoderRecenter);

    }

    @Override
    public void matchLoop() {}
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){
    }
}
