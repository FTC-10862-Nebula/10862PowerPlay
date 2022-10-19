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

    public static PIDFCoefficients pidfUpCoefficients = new PIDFCoefficients(0.01, 0.02, 0, 0);
//    public static PIDFCoefficients pidfDownCoefficients = new PIDFCoefficients(0.01, 0.00, 0, 0);

    //I = 0.0008
    public static double ARM_OFFSET = 0;
    private PIDFController upController, downController;
    private boolean automatic;

    public static double CPR = 751.8;
    public static double UP_SPEED = -0.2;
    public static double DOWN_SPEED = 0.2;

    private double encoderOffset = 0;
    private double encoderOffset2 = 0;

    public static int RESTING_POS = -2;
    public static int GROUND_POS = -200;
    public static int LOW_POS = -400;
    public static int MID_POS = -600;
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

        upController = new PIDFController(pidfUpCoefficients.p, pidfUpCoefficients.i, pidfUpCoefficients.d, pidfUpCoefficients.f, getAngle(), getAngle());
        upController.setTolerance(10);

//        downController = new PIDFController(pidfDownCoefficients.p, pidfDownCoefficients.i, pidfDownCoefficients.d, pidfDownCoefficients.f, getAngle(), getAngle());
//        downController.setTolerance(10);
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
            upController.setF(pidfUpCoefficients.f * Math.cos(Math.toRadians(upController.getSetPoint())));

            double output = upController.calculate(getAngle());

//            downController.setF(pidfDownCoefficients.f * Math.cos(Math.toRadians(downController.getSetPoint())));
//
//            output = downController.calculate(getAngle());

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
        upController.setSetPoint(getAngle());
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
//        downController.setSetPoint(RESTING_POS);
          upController.setSetPoint(RESTING_POS);

        liftPosition = 0;
    }

    public void encoderReset() {
        slideMotor1.resetEncoder();
        slideMotor2.resetEncoder();
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
        }
    }
}