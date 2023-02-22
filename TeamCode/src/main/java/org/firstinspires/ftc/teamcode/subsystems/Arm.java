package org.firstinspires.ftc.teamcode.subsystems;

public class Arm {
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
