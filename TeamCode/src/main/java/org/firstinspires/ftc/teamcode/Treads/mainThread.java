package org.firstinspires.ftc.teamcode.Treads;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ClawServos;
import org.firstinspires.ftc.teamcode.subsystems.Slide;

class Single implements Runnable {
    @Override
    public void run() {
       Slide.slideLow();
    }
}
class Test
{
    public static void main (){
        Thread t = new Thread();
        t.start();

    }
}


