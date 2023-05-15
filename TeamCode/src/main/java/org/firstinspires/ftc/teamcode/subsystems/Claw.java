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
    public final static double CLOSE_POS_S1 = 0.56,
                                AUTO_CLOSE_S1 = 0.5,
                                AUTO_OPEN_S1 = 0.24,
                                OPEN_POS_S1 = 0.2;



    Telemetry telemetry;
    private final ServoEx clawS1;     //Claw

    public Claw(Telemetry tl, HardwareMap hw) {
        this.clawS1 = new SimpleServo(hw, "clawS2", 0, 360);
        this.clawS1.setPosition(CLOSE_POS_S1);  //Port 3
//        this.clawS1.setInverted(true);

        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        telemetry.addData("Claw Servo 1 Pos: ", clawS1.getPosition());
    }

    public void setClawS1(double clawServo1Pos) {
        clawS1.setPosition(clawServo1Pos);
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