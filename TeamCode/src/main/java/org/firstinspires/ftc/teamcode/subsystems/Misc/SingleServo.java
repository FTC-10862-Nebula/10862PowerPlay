package org.firstinspires.ftc.teamcode.subsystems.Misc;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SingleServo extends SubsystemBase {
    //Claw Variables
//    public final static double CLOSE_POS_S1 = 0.485,
//                                AUTO_CLOSE_S1 = 0.477,
//                                OPEN_POS_S1 = 0.141;

    Telemetry telemetry;
    private final ServoEx servoOne;

    public SingleServo(HardwareMap hw,Telemetry tl) {

        this.servoOne = new SimpleServo(hw, "clawS3", 0, 360);
        this.servoOne.setPosition(0.5);  //Port 2

        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        telemetry.addData("Claw Servo 1 Pos: ", servoOne.getPosition());
    }

    public void set(double clawServo1Pos) {
        servoOne.setPosition(clawServo1Pos);
    }

}