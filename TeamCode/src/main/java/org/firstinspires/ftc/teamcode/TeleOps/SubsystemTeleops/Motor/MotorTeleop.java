package org.firstinspires.ftc.teamcode.TeleOps.SubsystemTeleops.Motor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.driveTrainAuton.MatchOpMode;
import org.firstinspires.ftc.teamcode.driveTrainAuton.SampleMecanumDriveCorrect;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Disabled
@Config
@TeleOp(name = "MotorTeleop")
public class MotorTeleop extends MatchOpMode {
    private static double startPoseX = 0;
    private static double startPoseY = 0;
    private static double startPoseHeading = 0;

    //Motors and Servos
    private MotorEx leftFront, leftRear, rightRear, rightFront;

    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;
    // Subsystems
    private Drivetrain drivetrain;
    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        drivetrain = new Drivetrain(new SampleMecanumDriveCorrect(hardwareMap), telemetry, hardwareMap);
        drivetrain.init();
    }

    //Buttons
    public Button intakeF, groundF, lowF, midF, highF;
    public Button intakeB, groundB, lowB, midB, highB;
    public Button clawMotorResetButton;
    public Button one, two;

    @Override
    public void configureButtons() {
//        one = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)
//                .whenPressed(Drivetrain.mecDrive(4,4,4,4))
//                .whenPressed(drive.setMotor));
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

