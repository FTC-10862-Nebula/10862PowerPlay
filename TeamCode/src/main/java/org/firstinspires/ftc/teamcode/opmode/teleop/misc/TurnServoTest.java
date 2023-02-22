package org.firstinspires.ftc.teamcode.opmode.teleop.misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.TurnServo;

//@Disabled
@TeleOp(group = "Subsystem test")
public class TurnServoTest extends OpMode {
    //hardware initialization stuff
    Servo servo;
    double pos = 0.84;

    /**
     * User defined init method
     * <p>oi
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "clawS3");
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        if(gamepad1.a){
            pos -= 0.001;
        }
        else if(gamepad1.b){
            pos += 0.001;
        }
        else if(gamepad1.dpad_down){
            pos = TurnServo.F_POS_S3;
        }
        else if(gamepad1.dpad_up){
            pos = TurnServo.B_POS_S3;
        }

        pos = Math.min(Math.max(pos, 0), 1);
        servo.setPosition(Math.min(Math.max(pos, 0), 1));
        telemetry.addData("Servo pos: ",servo.getPosition());
        telemetry.addData("Desired pos: ", pos);
        telemetry.update();
    }
}