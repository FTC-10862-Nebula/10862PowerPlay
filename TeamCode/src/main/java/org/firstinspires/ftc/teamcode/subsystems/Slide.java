package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.commands.Slide.SlideDefaultCommand;

import java.util.logging.Level;

@Config
public class Slide extends SubsystemBase {
    private Telemetry telemetry;
    private MotorEx slideM1;
    private MotorEx slideM2;

    public boolean liftTime;
    int liftError = 0, liftTargetPos = 0, setPos;

    public static PIDFCoefficients pidfUpCoefficients = new PIDFCoefficients(.005, 0.00, 0,0);//.0075, 0., .003, 0)
//    public static PIDFCoefficients pidfDownCoefficients = new PIDFCoefficients(0.01, 0.00, 0, 0);

    //I = 0.0008
    public static double ARM_OFFSET = 0;
    private PIDFController upController;//, downController;
    private boolean automatic;

    public static double CPR = 751.8;
    public static double UP_SPEED = -0.8;
    public static double DOWN_SPEED = 0.8;

    private double encoderOffset = 0;
    private double encoderOffset2 = 0;

    public static int RESTING_POS = 2;
    public static int GROUND_POS = -23;
    public static int LOW_POS = -670;
    public static int MID_POS = -1150;
    public static int HIGH_POS = -1330;

    //Auto Slide Positions
    public static int CONE_5_POS = -200;
    public static int CONE_4_POS = -200;
    public static int CONE_3_POS = -120;
    public static int CONE_2_POS = -100;
    public static int CONE_1_POS = -10;

    public static int CAP_POSITION = 0;

    private static double POWER = 0.98;

    private int liftPosition = 0;

    public Slide(MotorEx slideM1, MotorEx slideM2, Telemetry tl, HardwareMap hw) {
        this.slideM1 = slideM1;
        this.slideM2 = slideM2;

        this.slideM1 = new MotorEx(hw, "lift");
        this.slideM2 = new MotorEx(hw, "lift2");

        //Reverse lift motor
        this.slideM1.setInverted(true);
        //this.slideMotor2.setInverted(true);

        this.slideM1.resetEncoder();
        this.slideM2.resetEncoder();

        this.slideM1.setDistancePerPulse(360 / CPR);
        this.slideM2.setDistancePerPulse(360 / CPR);

        upController = new PIDFController(pidfUpCoefficients.p, pidfUpCoefficients.i, pidfUpCoefficients.d, pidfUpCoefficients.f, getAngle(), getAngle());
        upController.setTolerance(10);


        this.telemetry = tl;
        automatic = false;
        setOffset();
    }

    public void toggleAutomatic() {
        automatic = !automatic;
    }
    public boolean isAutomatic() {
        return automatic;
    }
    public void automaticFalse(){
        automatic=false;
    }

    @Override
    public void periodic() {
        if (automatic) {
            upController.setF(pidfUpCoefficients.f * Math.cos(Math.toRadians(upController.getSetPoint())));

            double output = upController.calculate(getAngle());
            if (output >= 1) output = 1;
            if (output <= -1) output = -1;

            slideM1.set(output* POWER);
            slideM2.set(output* POWER);
        }

        Util.logger(this, telemetry, Level.INFO, "lift encoder pos 1: ", slideM1.getCurrentPosition());
        Util.logger(this, telemetry, Level.INFO, "lift encoder pos 2: ", slideM2.getCurrentPosition());
    }

    private double getEncoderDistance() {
        return slideM1.getDistance() - encoderOffset;
    }

    private double getEncoderDistance2(){
        return slideM2.getDistance() -encoderOffset2;
    }

//    public void upSlideManual(GamepadEx operatorGamepad, Slide slide) {
//        new SlideDefaultCommand(slide, operatorGamepad);
//        automatic = false;
//    }
    public void upSlideManual(){
        automatic = false;
        slideM1.set(UP_SPEED);
        slideM2.set(UP_SPEED);
    }

    public void setPower(double power)
    {
        slideM1.set(power);
        slideM2.set(power);
    }

    public void downSlideManual() {
        automatic = false;
        slideM1.set(DOWN_SPEED);
        slideM2.set(DOWN_SPEED);
    }

    public void stopSlide() {
        slideM1.stopMotor();
        upController.setSetPoint(getAngle());
        slideM2.stopMotor();
        automatic = false;
    }

    public void setAutomatic(boolean auto) {
        this.automatic = auto;
    }

    public void resetEncoder() {
        liftEncoderReset();
    }

    public double getAngle() {
        return getEncoderDistance();
    }

    public double getAngle2(){
        return getEncoderDistance2();
    }

    /****************************************************************************************/


    public void slideResting() {
        automatic = true;
        upController.setSetPoint(RESTING_POS);
        liftPosition = 0;
//      downController.setSetPoint(RESTING_POS);
    }

    public void encoderReset() {
        slideM1.resetEncoder();
        slideM2.resetEncoder();
    }

    public void slideGround() {
        automatic = true;
        upController.setSetPoint(GROUND_POS);
        liftPosition = 1;
    }
    public void slideLow() {
        automatic = true;
        upController.setSetPoint(LOW_POS);
        liftPosition = 2;
    }
    public void slideMid() {
        automatic = true;
        upController.setSetPoint(MID_POS);
        liftPosition = 3;
    }
    public void slideHigh() {
        automatic = true;
        upController.setSetPoint(HIGH_POS);
        liftPosition = 4;
    }
    public void slideCone5() {
        automatic = true;
        upController.setSetPoint(CONE_5_POS);
        liftPosition = 5;
    }
    public void slideCone4() {
        automatic = true;
        upController.setSetPoint(CONE_4_POS);
        liftPosition = 6;
    }
    public void slideCone3() {
        automatic = true;
        upController.setSetPoint(CONE_3_POS);
        liftPosition = 7;
    }
    public void slideCone2() {
        automatic = true;
        upController.setSetPoint(CONE_2_POS);
        liftPosition = 8;
    }
    public void slideCone1() {
        automatic = true;
        upController.setSetPoint(CONE_1_POS);
        liftPosition = 9;
    }
    public void autoPickSlideUp() {
        automatic = true;
        upController.setSetPoint(slideM1.getCurrentPosition()+150);
        liftPosition = 10;
    }
    public void autoDropSlideUp() {
        automatic = true;
        upController.setSetPoint(slideM1.getCurrentPosition()-150);
        liftPosition = 11;
    }



    public void liftEncoderReset() {
        liftPosition = 0;
    }

    public void setLift(double angle) {
        automatic = true;
        upController.setSetPoint(angle);
    }

    public boolean atTargetAngle() {
        return upController.atSetPoint();
    }


    public void setOffset() {
        resetEncoder();
        upController.setSetPoint(getAngle());
    }

//    public void moveUp() {
//        liftPosition = liftPosition + 1;
//        if(liftPosition > 4) {
//            liftPosition = 4;
//        }
//        moveLiftToCorrectHeight();
//    }
//
//    public void moveDown() {
//        liftPosition = liftPosition - 1;
//        if(liftPosition < 0) {
//            liftPosition = 0;
//        }
//        moveLiftToCorrectHeight();
//    }

    public void moveLiftToCorrectHeight() {
        if(liftPosition == 0) {
            slideResting();
        } else if(liftPosition == 1) {
            slideGround();
        } else if(liftPosition == 2) {
            slideLow();
        } else if(liftPosition == 3) {
            slideMid();
        } else if(liftPosition == 4) {
            slideHigh();
        } else if(liftPosition == 5) {
            slideCone5();
        } else if(liftPosition == 6) {
            slideCone4();
        } else if(liftPosition == 7) {
            slideCone3();
        } else if(liftPosition == 8) {
            slideCone2();
        } else if(liftPosition == 9) {
            slideCone1();
        } else if(liftPosition == 10) {
            autoPickSlideUp();
        } else if(liftPosition == 11) {
            autoDropSlideUp();
        }
    }
}