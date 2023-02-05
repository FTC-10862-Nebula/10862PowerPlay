package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.logging.Level;

@Config
public class Arm extends SubsystemBase {

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(.005, 0.00, 0.0,0);
    private final PIDFController controller;
    private boolean armAutomatic;
    public boolean shouldSensorWork = true;


    public static double CPR = 384.5;
    public static double UP_SPEED = -0.55;
    public static double DOWN_SPEED = 0.55;

    private final double encoderOffset = 0;
    public int INIT_POS = 0;

    public static int INTAKE_POS_BACK = -220,
                        POS_BACK = -200,
                        HIGH_POS_BACK = -113 ,
                        GROUND_POS_BACK = -197,
    DROP_BACK = -398;
    public static int HIGH_POS_AUTO_BACK = -115,
                        INTAKE_POS_AUTO_BACK = -223,
                        POS_AUTO_BACK = -140;

    public static int INTAKE_POS_FRONT = -INTAKE_POS_BACK,
                        POS_FRONT = -POS_BACK,
                        HIGH_POS_FRONT = -HIGH_POS_BACK,
                        GROUND_POS_FRONT = -GROUND_POS_BACK,
    DROP_FRONT = -DROP_BACK;
    public static int HIGH_POS_AUTO_FRONT = -HIGH_POS_AUTO_BACK,
                        INTAKE_POS_AUTO_FRONT = -INTAKE_POS_AUTO_BACK,
                        POS_AUTO_FRONT = -POS_AUTO_BACK;
    public enum ArmPos{
        RESET,
        INTAKE_BACK, BACK, HIGH_BACK, GROUND_BACK, DROP_BACK,
        INTAKE_FRONT, FRONT, HIGH_FRONT, GROUND_FRONT, DROP_FRONT,
        AUTO_INTAKE_BACK, AUTO_BACK, AUTO_HIGH_BACK,
        AUTO_INTAKE_FRONT, AUTO_FRONT, AUTO_HIGH_FRONT,
    }
    ArmPos armPos = ArmPos.RESET;



    private final static double POWER = 0.93;
//    private static int armPos = 0;

    Telemetry telemetry;
    private final MotorEx armMotor;

    public Arm(Telemetry tl, HardwareMap hw) {
        armMotor = new MotorEx(hw, "clawM");

        //Reverse claw motor
        armMotor.setInverted(true);
        armMotor.resetEncoder();
//        this.armMotor.setZeroPowerBehavior(BRAKE); // TODO: FOR COMP
        armMotor.setDistancePerPulse(360 / CPR);

        armMotor.set(0);

        controller = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f, getAngle(), getAngle());
        controller.setTolerance(10);

        this.telemetry = tl;
        armAutomatic = false;
        setOffset();
        //TODO: isAuto for Brake Mode
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE); //Remove for practice and stuff
//        armPos = ArmPos.RESET;
//        potentiometer = hw.get(AnalogInput.class, "Potentiometer");
//        potentiometer = new AnalogInput(null, 0);
    }

    @Override
    public void periodic() {
        if (armAutomatic) {

            controller.setF(pidfCoefficients.f * Math.cos(Math.toRadians(controller.getSetPoint())));

            double output = controller.calculate(getAngle());
            telemetry.addData("CLaw Motor Output:", output);

//            if (output >= 1) output = 1;
//            if (output <= -1) output = -1;
//            armMotor.set(output);

            armMotor.set(output * POWER);

        }
        Util.logger(this, telemetry, Level.INFO, "Arm Encoder Pos: ", armMotor.getCurrentPosition());
        Util.logger(this, telemetry, Level.INFO, "Arm Pos: ", armPos);
//        telemetry.addData("Current Voltage", potentiometer.getVoltage());

    }


    private double getEncoderDistance() {
        return armMotor.getDistance() - encoderOffset;
    }
    public double getAngle() {
        return getEncoderDistance();
    }

    /****************************************************************************************/

    public void raiseClawManual() {
        armAutomatic = false;
        armMotor.set(UP_SPEED);

//        armAutomatic = true;
////        if((armMotor.getCurrentPosition()<275)){
//            controller.setSetPoint(armMotor.getCurrentPosition()+5);
////        }
////        else return;
    }
    public void lowerClawManual() {
        armAutomatic = false;
        armMotor.set(DOWN_SPEED);

//        armAutomatic = true;
//        if((armMotor.getCurrentPosition()>-275)){
//            controller.setSetPoint(armMotor.getCurrentPosition()-5);
//        }
//        else return;
    }

    public void stopArm() {
        armMotor.stopMotor();
//        controller.setSetPoint(getAngle());
        armAutomatic = false;
    }

    /****************************************************************************************/

    public void moveReset(){
        armAutomatic = true;
        controller.setSetPoint(INIT_POS);
        armPos = ArmPos.RESET;
        shouldSensorWork = true;
    }
    public void moveIntakeF() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_FRONT);
        armPos = ArmPos.INTAKE_FRONT;
        shouldSensorWork = true;
    }
    public void moveIntakeB() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_BACK);
        armPos = ArmPos.INTAKE_BACK;
        shouldSensorWork = true;
    }
    public void moveGroundB(){
        armAutomatic = true;
        controller.setSetPoint(GROUND_POS_BACK);
        armPos = ArmPos.GROUND_BACK;
        shouldSensorWork = false;
    }
    public void moveGroundF(){
        armAutomatic = true;
        controller.setSetPoint(GROUND_POS_FRONT);
        armPos = ArmPos.GROUND_FRONT;
        shouldSensorWork = false;
    }
    public void moveF() {
        armAutomatic = true;
        controller.setSetPoint(POS_FRONT);
        armPos = ArmPos.FRONT;
        shouldSensorWork = false;
    }
    public void moveB() {
        armAutomatic = true;
        controller.setSetPoint(POS_BACK);
        armPos = ArmPos.BACK;
        shouldSensorWork = false;
    }
    public void moveHighF() {
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_FRONT);
        armPos = ArmPos.HIGH_FRONT;
        shouldSensorWork = false;
    }
    public void moveHighB() {
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_BACK);
        armPos = ArmPos.HIGH_BACK;
        shouldSensorWork = false;
    }



    public void moveHighBAuto(){
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_AUTO_BACK);
        armPos = ArmPos.AUTO_HIGH_BACK;
        shouldSensorWork = false;
    }
    public void moveHighFAuto(){
        armAutomatic = true;
        controller.setSetPoint(HIGH_POS_AUTO_FRONT);
        armPos = ArmPos.AUTO_HIGH_FRONT;
        shouldSensorWork = false;
    }
    public void moveIntakeFAuto() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_AUTO_FRONT);
        armPos = ArmPos.AUTO_INTAKE_FRONT;
        shouldSensorWork = true;
    }
    public void moveIntakeBAuto() {
        armAutomatic = true;
        controller.setSetPoint(INTAKE_POS_AUTO_BACK);
        armPos = ArmPos.AUTO_INTAKE_BACK;
        shouldSensorWork = true;
    }
    public void moveBAuto() {
        armAutomatic = true;
        controller.setSetPoint(POS_AUTO_BACK);
        armPos = ArmPos.AUTO_BACK;
        shouldSensorWork = false;
    }
    public void moveFAuto() {
        armAutomatic = true;
        controller.setSetPoint(POS_AUTO_FRONT);
        armPos = ArmPos.AUTO_FRONT;
        shouldSensorWork = false;
    }
    public void moveBDrop() {
        armAutomatic = true;
        controller.setSetPoint(DROP_BACK);
        armPos = ArmPos.DROP_BACK;
        shouldSensorWork = false;
    }
    public void moveFDrop() {
        armAutomatic = true;
        controller.setSetPoint(DROP_FRONT);
        armPos = ArmPos.DROP_FRONT;
        shouldSensorWork = false;
    }


    public void dropArmTeleop(){
        switch (armPos){
            case FRONT:
                controller.setSetPoint(POS_FRONT+50);
                return;
            case BACK:
                controller.setSetPoint(POS_BACK-50);
                return;


            case HIGH_BACK:
//                controller.setSetPoint(HIGH_POS_BACK-120);
                controller.setSetPoint(DROP_BACK    );
                return;
            case HIGH_FRONT:
//                controller.setSetPoint(HIGH_POS_FRONT+120);
                controller.setSetPoint(DROP_FRONT);
                return;
        }
    }

    public void dropArmAuto(){
        switch (armPos){
            case AUTO_HIGH_BACK:
                controller.setSetPoint(HIGH_POS_AUTO_BACK-110);
                return;
            case AUTO_HIGH_FRONT:
                controller.setSetPoint(HIGH_POS_AUTO_FRONT+110);
                return;

            case AUTO_BACK:
                controller.setSetPoint(POS_AUTO_BACK-35);
                return;
            case AUTO_FRONT:
                controller.setSetPoint(POS_AUTO_FRONT+35);
                return;

            case AUTO_INTAKE_FRONT:
                controller.setSetPoint(INTAKE_POS_AUTO_FRONT+25);
                shouldSensorWork = true;
                return;
            case AUTO_INTAKE_BACK:
                controller.setSetPoint(INTAKE_POS_AUTO_BACK-25);
                shouldSensorWork = true;
                return;
        }
    }

    public void encoderReset() {
        armMotor.resetEncoder();
        telemetry.addLine("ARM RESET");
    }

    /****************************************************************************************/


    public void setOffset() {
        resetEncoder();
        controller.setSetPoint(getAngle());
    }
    public void clawEncoderReset() {
        armPos = ArmPos.RESET;
    }
    public void resetEncoder() {
        clawEncoderReset();
    }

    //Check Documentation if confused
//    public double getPotentiometerAngle(){
//        double angle = potentiometer.getVoltage()*81.8;
//        return Range.scale(potentiometer.getVoltage(), 0, potentiometer.getMaxVoltage(), 0, 270);
//    }

    public void setArm(int point){
        controller.setSetPoint(point);
    }
}
