package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

@Config
public class Slide extends SubsystemBase {
    private Telemetry telemetry;
    private MotorEx slideMotor1;
    private MotorEx slideMotor2;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.5, 0.05, 0, 0);
    //I = 0.0008
    public static double ARM_OFFSET = 0;
    private PIDFController controller;
    private boolean automatic;

    public static double CPR = 751.8;
    public static double UP_SPEED = -0.25;
    public static double DOWN_SPEED = 0.25;

    private double encoderOffset = 0;
    private double encoderOffset2 = 0;

    public static int RESTING_POS = -10;
    public static int GROUND_POS = -200;
    public static int LOW_POS = -300;
    public static int MID_POS = -400;
    public static int HIGH_POS = -1000;

    //Auto Slide Positions
    public static int CONE_5_POS = -100;
    public static int CONE_4_POS = -100;
    public static int CONE_3_POS = -100;
    public static int CONE_2_POS = -100;
    public static int CONE_1_POS = -100;

    public static int CAP_POSITION = 0;


    private int liftPosition = 0;

    public Slide(MotorEx slideMotor1, MotorEx slideMotor2, Telemetry tl, HardwareMap hw) {
        this.slideMotor1 = slideMotor1;
        this.slideMotor2 = slideMotor2;

        this.slideMotor1 = new MotorEx(hw, "lift");
        this.slideMotor2 = new MotorEx(hw, "lift2");

        //Reverse lift motor
        this.slideMotor1.setInverted(true);
        //this.slideMotor2.setInverted(true);

        this.slideMotor1.resetEncoder();
        this.slideMotor2.resetEncoder();

        this.slideMotor1.setDistancePerPulse(360 / CPR);
        this.slideMotor2.setDistancePerPulse(360 / CPR);

        controller = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f, getAngle(), getAngle());
        controller.setTolerance(10);

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

    @Override
    public void periodic() {
        if (automatic) {
            controller.setF(pidfCoefficients.f * Math.cos(Math.toRadians(controller.getSetPoint())));

            double output = controller.calculate(getAngle());

            slideMotor1.set(output);
            slideMotor2.set(output);
        }
        Util.logger(this, telemetry, Level.INFO, "lift encoder pos 1: ", slideMotor1.getCurrentPosition());
        Util.logger(this, telemetry, Level.INFO, "lift encoder pos 2: ", slideMotor2.getCurrentPosition());
    }

    private double getEncoderDistance() {
        return slideMotor1.getDistance() - encoderOffset;
    }

    private double getEncoderDistance2(){
        return slideMotor2.getDistance() -encoderOffset2;
    }

    public void upSlideManual() {
        automatic = false;
        slideMotor1.set(UP_SPEED);
        slideMotor2.set(UP_SPEED);
    }

    public void downSlideManual() {
        automatic = false;
        slideMotor1.set(DOWN_SPEED);
        slideMotor2.set(DOWN_SPEED);
    }

    public void stopSlide() {
        slideMotor1.stopMotor();
        controller.setSetPoint(getAngle());
        slideMotor2.stopMotor();
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
        controller.setSetPoint(RESTING_POS);
        liftPosition = 0;
    }

    public void encoderReset() {
        slideMotor1.resetEncoder();
        slideMotor2.resetEncoder();
    }

    public void slideGround() {
        automatic = true;
        controller.setSetPoint(GROUND_POS);
        liftPosition = 1;
    }
    public void slideLow() {
        automatic = true;
        controller.setSetPoint(LOW_POS);
        liftPosition = 2;
    }
    public void slideMid() {
        automatic = true;
        controller.setSetPoint(MID_POS);
        liftPosition = 3;
    }
    public void slideHigh() {
        automatic = true;
        controller.setSetPoint(HIGH_POS);
        liftPosition = 4;
    }
    public void slideCone5() {
        automatic = true;
        controller.setSetPoint(CONE_5_POS);
        liftPosition = 5;
    }
    public void slideCone4() {
        automatic = true;
        controller.setSetPoint(CONE_4_POS);
        liftPosition = 6;
    }
    public void slideCone3() {
        automatic = true;
        controller.setSetPoint(CONE_3_POS);
        liftPosition = 7;
    }
    public void slideCone2() {
        automatic = true;
        controller.setSetPoint(CONE_2_POS);
        liftPosition = 8;
    }
    public void slideCone1() {
        automatic = true;
        controller.setSetPoint(CONE_1_POS);
        liftPosition = 9;
    }


    public void liftEncoderReset() {
        liftPosition = 0;
    }

    public void setLift(double angle) {
        automatic = true;
        controller.setSetPoint(angle);
    }

    public boolean atTargetAngle() {
        return controller.atSetPoint();
    }


    public void setOffset() {
        resetEncoder();
        controller.setSetPoint(getAngle());
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
        }
    }
}