package org.firstinspires.ftc.teamcode.subsystems;

public class Arm {
    public static class State {
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
            INTAKE(0,0),
            GROUND(0,0),
            LOW(0,0),
            MID(0,0),
            HIGH(0,0),
            MID_AUTO(0,0),
            HIGH_AUTO(0,0),
            CONE5(0,0),
            CONE4(0,0),
            CONE3(0,0),
            CONE2(0,0),
            CONE1(0,0),
            ;
            public final double pivotPosition;
            public final double slidePosition;
            Position(double pivotPosition, double slidePosition) {
                this.pivotPosition = pivotPosition;
                this.slidePosition = slidePosition;
            }
        }
    }
    public final Slide slide;
    public final Pivot pivot;
    public final Claw claw;
    public final TurnServo turnServo;
    public Arm(Slide slide, Pivot pivot, Claw claw, TurnServo turnServo) {
        this.slide = slide;
        this.pivot = pivot;
        this.claw = claw;
        this.turnServo = turnServo;
    }
}
