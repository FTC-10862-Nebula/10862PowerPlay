package org.firstinspires.ftc.teamcode.TeleOps.SubsystemTeleops.Drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommands.TeleopCommands.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDriveCorrect;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainCOrrect;

@Disabled

@Config
@TeleOp(name = "OfficialDriveOnlyTeleop")
public class OfficialDriveOnlyTel extends MatchOpMode {

    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx leftFront, leftRear, rightRear, rightFront;
    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;
    // Subsystems
    private DrivetrainCOrrect drivetrainCorrect;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        drivetrainCorrect = new DrivetrainCOrrect(new SampleMecanumDriveCorrect(hardwareMap), telemetry, hardwareMap);
        drivetrainCorrect.init();
        drivetrainCorrect.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
        drivetrainCorrect.setDefaultCommand(new DefaultDriveCommand(drivetrainCorrect, driverGamepad, false));

//        standardTrackingWheelLocalizer = new StandardTrackingWheelLocalizer(leftEncoder, rightEncoder, frontEncoder, hardwareMap);
    }

    //Buttons
//    private Button slowModeBumper, robotDriveButton, fieldDriveButton;


    @Override
    public void configureButtons() {
        //Robot/Centric Drive Button
        Button robotDriveButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.START))
                .whenPressed(new DefaultDriveCommand(drivetrainCorrect,driverGamepad,  true));
        Button fieldDriveButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.BACK))
                .whenPressed(new DefaultDriveCommand(drivetrainCorrect,driverGamepad,false));

        //Slowmode - D1
            Button slowModeBumper = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER))
                    .whileHeld(new SlowDriveCommand(drivetrainCorrect, driverGamepad));

    }

    @Override
    public void matchLoop() { }
    @Override
    public void disabledPeriodic() { }
    @Override
    public void matchStart() { }
    @Override
    public void robotPeriodic(){}
}
