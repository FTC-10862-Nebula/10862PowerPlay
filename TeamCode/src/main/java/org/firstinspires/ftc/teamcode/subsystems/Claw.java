package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Claw extends SubsystemBase {
    //Claw Variables
    public final static double CLOSE_POS_S1 = 0.68,
                                AUTO_CLOSE_S1 = 0.85,
                                OPEN_POS_S1 = 0.88;

//    private static double INTAKE_POWER = -1;
//    private static double OUTTAKE_POWER = 1;

    public static final double F_POS_S3 = 0.07,
            B_POS_S3 = 0.73499;//299;


    Telemetry telemetry;
    private final ServoEx clawS1, //0. - 0.78
                    clawS3;     //Servo that turns claw
//    public Claw( HardwareMap hw, Telemetry tl) { - //Change ITIITITITIIT
    public Claw(Telemetry tl, HardwareMap hw) {

//        this.clawS1 = new SimpleServo(hw, "clawS1", 0, 360);
//        this.clawS1.setPosition(CLOSE_POS_S1);  //Port 2
        this.clawS1 = new SimpleServo(hw, "clawS2", 0, 360);
        this.clawS1.setPosition(CLOSE_POS_S1);  //Port 3

        this.clawS3 = new SimpleServo(hw, "clawS3", 0, 360);
        this.clawS3.setPosition(F_POS_S3);      //Port 5

        this.telemetry = tl;
    }

    @Override
    public void periodic() {
//        telemetry.addData("Claw Servo 1 Pos: ", clawS1.getPosition());
        telemetry.addData("Claw Servo 1 Pos: ", clawS1.getPosition());
        telemetry.addData("Claw Servo 3 Pos: ", clawS3.getPosition());
    }

//    public void setClawS1(double clawServo1Pos) {
//        clawS1.setPosition(clawServo1Pos);
//    }
    public void setClawS1(double clawServo1Pos) {
        clawS1.setPosition(clawServo1Pos);
    }
    public void setClawS3(double clawServo3Pos) {
        clawS3.setPosition(clawServo3Pos);
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


    /*public void intakeClaw() {
        clawS2.set(INTAKE_POWER);
    }
    public void outtakeClaw() {
        clawS2.set(OUTTAKE_POWER);
    }
    public void stopClaw() {
        clawS2.stop();
    public void addClawPos() {
        if (clawS1.getPosition()>(CLOSE_POS_S1+0.05) && clawS2.getPosition()>(CLOSE_POS_S2+0.05)) {
            setClawS1(clawS1.getPosition() + 0.05);
        }
//        else return;
    }
    public void subClawPos() {
        if (clawS1.getPosition()<(OPEN_POS_S1-0.05) && clawS2.getPosition()>(CLOSE_POS_S2-0.05)) {
            setClawS1(clawS1.getPosition() - 0.05);
        }
//        else return;
    }*/

    public void setFClawPos(){setClawS3(F_POS_S3);}
    public void setBClawPos(){setClawS3(B_POS_S3);}

    /*public void addClaw3Pos() {
        setClawS3(clawS3.getPosition() + 0.05);
    }
    public void subClaw3Pos() {
        setClawS3(clawS3.getPosition() - 0.05);
    }*/
}