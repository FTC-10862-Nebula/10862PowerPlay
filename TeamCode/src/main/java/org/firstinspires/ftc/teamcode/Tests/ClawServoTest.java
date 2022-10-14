package org.firstinspires.ftc.teamcode.Tests;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ClawServoTest")
public class ClawServoTest extends OpMode {
    //hardware initialization stuff
    Servo servo1;
    CRServo clawS2;
    double pos = 0.6;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "clawS1");
        clawS2 = hardwareMap.get(CRServo.class, "clawS2");
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


        if(gamepad1.right_bumper){
            clawS2.set(1);
        }
        else if(gamepad1.left_bumper){
            clawS2.set(-1);
        }
        else{
            clawS2.stop();
        }


        pos = Math.min(Math.max(pos, 0), 1);
        servo1.setPosition(Math.min(Math.max(pos, 0), 1));
        telemetry.addData("Servo pos: ",servo1.getPosition());
        telemetry.addData("Desired pos: ", pos);
        telemetry.update();

    }
}