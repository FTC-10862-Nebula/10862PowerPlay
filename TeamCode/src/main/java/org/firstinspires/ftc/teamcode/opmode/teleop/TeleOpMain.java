package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadTrigger;
import org.firstinspires.ftc.teamcode.commands.sensor.Sensor;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp
public class TeleOpMain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(this);
        Claw gripper = new Claw(this);
        Drive drive = new Drive(this,true);
        Arm arm = new Arm(this);

        Sensor sensor = new Sensor(this, arm, lift, gripper);

        drive.setPoseEstimate(PoseStorage.currentPose);
        gripper.open();
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            lift.runIteratively();
            arm.runIteratively();
            drive.runIteratively();
            sensor.runIteratively();
            gripper.runIteratively();

            telemetry.update();
        }

    }
}
