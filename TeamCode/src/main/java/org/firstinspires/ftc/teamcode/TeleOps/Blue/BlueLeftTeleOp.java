package org.firstinspires.ftc.teamcode.TeleOps.Blue;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOps.ButtonCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.SensorCommands.Teleop.BlueIntakeTeleopCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.subsystems.Slide;
import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

@Config
@TeleOp(name = "BLUE Left")
public class BlueLeftTeleOp extends MatchOpMode {
    int choice = 1;
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

//        sensorColor = new SensorColor(hardwareMap, telemetry);
//        claw.setDefaultCommand(new BlueIntakeTeleopCommand(slide, claw, sensorColor,arm));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad, false, choice));
    }


    @Override
    public void configureButtons() {
        new ButtonCommand(driverGamepad, operatorGamepad, drivetrain,  arm, slide, turnServo, claw, choice);
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
