package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ArmMoveToPosition extends SequentialCommandGroup {
    public static class ArmState {
        public enum GameMode {
            AUTO,
            TELEOP
        }
        public enum Side {
            FRONT(1, 0.049),
            BACK(-1, 0.708),
            ;

            public final double multiplier;
            public final double turnServoPosition;

            Side(double pivotMultiplier, double turnServoPosition) {
                this.multiplier = pivotMultiplier;
                this.turnServoPosition = turnServoPosition;
            }
        }
        public enum Position {
            INTAKE(0,0,0),
            GROUND(0,0,0),
            LOW(0,0,0),
            MID(0,0,0),
            HIGH(0,0,0),
            CONE5(0,0,0),
            CONE4(0,0,0),
            CONE3(0,0,0),
            CONE2(0,0,0),
            CONE1(0,0,0),
            ;
            public final double pivotPosition;
            public final double slideAutoPosition;
            public final double slideTeleOpPosition;
            Position(double pivotPosition, double slideAutoPosition, double slideTeleOpPosition) {
                this.pivotPosition = pivotPosition;
                this.slideAutoPosition = slideAutoPosition;
                this.slideTeleOpPosition = slideTeleOpPosition;

            }
        }
    }
    public ArmMoveToPosition(Arm arm, ArmState.GameMode gameMode, ArmState.Side side, ArmState.Position position) {
        addRequirements(arm.slide, arm.claw, arm.pivot, arm.turnServo);

    }
}
