package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

@Config
public class ClawServos extends SubsystemBase {
    //Claw Variables
    public static double CLOSE_POS_S1 = 0.85;
    public static double CLOSE_POS_S2 = 0.;
    //Servo 1 - No more than 0.71 and no less than 0.9

    public static double OPEN_POS_S1 = 0.7;
    public static double OPEN_POS_S2 = 0.;
    //Servo 2 - intake is less and outtake is more

    public static double FRONT_POS_S3 = 0.05;
    public static double BACK_POS_S3 = 0.78;


    Telemetry telemetry;
    private ServoEx clawS1;
    private ServoEx clawS2;
    private ServoEx clawS3;     //Servo that turns claw

    public ClawServos(ServoEx clawS1, ServoEx clawS2, ServoEx clawS3, Telemetry tl, HardwareMap hw) {
        this.clawS1 = clawS1;
        this.clawS2 = clawS2;
        this.clawS3 = clawS3;


        this.clawS1 = new SimpleServo(hw,"clawS1",0,360);
        this.clawS1.setPosition(CLOSE_POS_S1);
        this.clawS2 = new SimpleServo(hw,"clawS2",0,360);
        this.clawS2.setPosition(CLOSE_POS_S2);
        this.clawS3 = new SimpleServo(hw,"clawS3",0,360);
        this.clawS3.setPosition(FRONT_POS_S3);

        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        Util.logger(this, telemetry, Level.INFO, "Claw Servo 1 Pos: ", clawS1.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Claw Servo 2 Pos: ", clawS2.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Claw Servo 3 Pos: ", clawS3.getPosition());

    }


    public void setClawS1(double clawServo1Pos) {
        clawS1.setPosition(clawServo1Pos);
    }
    public void setClawS2(double clawServo2Pos) {
        clawS2.setPosition(clawServo2Pos);
    }
    public void setClawS3(double clawServo3Pos) {
        clawS2.setPosition(clawServo3Pos);
    }

    public void clawClose() {
        setClawS1(CLOSE_POS_S1);
        setClawS2(CLOSE_POS_S2);
    }
    public void clawOpen() {
        setClawS1(OPEN_POS_S1);
        setClawS2(OPEN_POS_S2);
    }


    public void addClawPos() {
        setClawS1(clawS1.getPosition()+0.05);
        setClawS2(clawS1.getPosition()+0.05);
    }
    public void subClawPos() {
        setClawS1(clawS1.getPosition()-0.05);
        setClawS2(clawS2.getPosition()-0.05);
    }

    public void addClaw3Pos() {
        setClawS3(clawS3.getPosition()+0.05);
    }
    public void subClaw3Pos() {
        setClawS3(clawS3.getPosition()-0.05);
    }
    public void orgClawPos(){
        setClawS3(FRONT_POS_S3);
    }

}
