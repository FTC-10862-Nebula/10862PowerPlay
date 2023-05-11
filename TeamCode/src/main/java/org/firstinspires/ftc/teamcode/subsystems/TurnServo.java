package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TurnServo extends SubsystemBase {
    public static final double F_POS_S3 = 0.01,
            B_POS_S3 = 0.7;
    Telemetry telemetry;
    private final ServoEx clawS3;     //Servo that turns claw

        public TurnServo(Telemetry tl, HardwareMap hw) {
        this.clawS3 = new SimpleServo(hw, "clawS3", 0, 360);
        this.clawS3.setPosition(F_POS_S3);      //Port 5

            this.clawS3.setInverted(false );
        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        telemetry.addData("Claw Servo 3 Pos: ", clawS3.getPosition());
    }

    public void setClawS3(double clawServo3Pos) {
        clawS3.setPosition(clawServo3Pos);
    }public double getPos() {
        return clawS3.getPosition();
    }


    public void setFClawPos() {
        setClawS3(F_POS_S3);
    }

    public void setBClawPos() {
        setClawS3(B_POS_S3);
    }
}