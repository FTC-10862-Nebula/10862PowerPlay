package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Config
@TeleOp(name = "RED Up")
public class RedTeleOp extends MatchOpMode {
    int choice = 2;
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

//        sensorColor = new SensorColor(hardwareMap, telemetry);
//
//        claw.setDefaultCommand(new RedIntakeTeleopCommand(slide, claw, sensorColor,arm));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false));
    }


    @Override
    public void configureButtons() {
        new ConfigureButton(driverGamepad, operatorGamepad, drivetrain,  arm, slide, turnServo, claw);
    }

    @Override
    public void matchLoop() {
    }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){}
}
