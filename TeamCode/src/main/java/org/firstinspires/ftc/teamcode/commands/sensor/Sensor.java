package org.firstinspires.ftc.teamcode.commands.sensor;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

public class Sensor extends Command {
    private enum SensorState {
        DISABLED,
        GRAB,
        LIFT
    }

    public static class Constants {
        public static double LIFT_WAIT_SECONDS = 0.3;
    }

    private final Slide lift;
    private final Arm arm;
    private final Claw claw;


    private SensorState sensorState = SensorState.GRAB;
    private final ElapsedTime waitTimer = new ElapsedTime();


    public Sensor(@NonNull LinearOpMode opMode, Arm arm, Lift lift, Claw claw) {
        super(opMode);
        this.arm = arm;
        this.lift = lift;
        this.claw = claw;

    }

    @Override
    protected void run() {
        switch (sensorState) {
            case DISABLED:
                if ((claw.ignoreSensor())) break;
                sensorState = SensorState.GRAB;
                break;
            case GRAB:
                if (!claw.isInRange()) break;
                if (!lift.isDown()) break;
                claw.close();
                waitTimer.reset();
                sensorState = SensorState.LIFT;
            case LIFT:
                if (!(waitTimer.seconds() > Constants.LIFT_WAIT_SECONDS)) break;
                arm.moveReset();
                sensorState = SensorState.DISABLED;
                break;
        }
    }
}
