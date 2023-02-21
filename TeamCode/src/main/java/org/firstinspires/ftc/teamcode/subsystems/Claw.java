package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Claw extends Subsystem {
    //Claw Variables
    public final static double CLOSE_POS_S1 = 0.1,
                                AUTO_CLOSE_S1 = 0.12,
                                AUTO_OPEN_S1 = 0.4,
                                OPEN_POS_S1 = 0.42;



    Telemetry telemetry;
    private final Servo clawS1;     //Claw
    private final ColorRangeSensor distanceSensor;

    private boolean ignoreSensor = false;
    public double SENSOR_DISTANCE = 35;


    public Claw(@NonNull OpMode opMode) {
        super(opMode);
        clawS1 = opMode.hardwareMap.get(Servo.class, "claws1");
        distanceSensor = opMode.hardwareMap.get(ColorRangeSensor.class, "distanceSensor");
        this.clawS1.setPosition(CLOSE_POS_S1);  //Port 3
    }

    @Override
    public void periodic() {
        telemetry.addData("Claw Servo 1 Pos: ", clawS1.getPosition());
    }

    public void setClawS1(double clawServo1Pos) {
        clawS1.setPosition(clawServo1Pos);
    }

    protected void manualControl() {
        ignoreSensor = true;
        if (opMode.gamepad2.b) open();
        else if (opMode.gamepad2.a) close();
        else ignoreSensor = false;
    }

    public void open() {
        clawS1.setPosition(OPEN_POS_S1);
    }

    public void close() {
        clawS1.setPosition(CLOSE_POS_S1);
    }

    public boolean ignoreSensor() {
        return ignoreSensor;
    }

    public boolean isInRange() {
        double distance = distanceSensor.getDistance(DistanceUnit.MM);
        opMode.telemetry.addData("Distance", distance);
        return (distance) < SENSOR_DISTANCE;
    }

    public void clawAutoClose() {
        setClawS1(AUTO_CLOSE_S1);
    }
    public void clawClose() {
        setClawS1(CLOSE_POS_S1);
    }

    public void clawOpen() {
        setClawS1(OPEN_POS_S1);
    }
    public void clawAutoOpen() {
        setClawS1(AUTO_OPEN_S1);
    }

    public boolean isClawOpen(){
//        return clawS1.getPosition()==OPEN_POS_S1;
        return (clawS1.getPosition()==CLOSE_POS_S1) || (clawS1.getPosition()==AUTO_CLOSE_S1);
    };

}