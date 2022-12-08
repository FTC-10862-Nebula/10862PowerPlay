package org.firstinspires.ftc.teamcode.Treads;

public class mainThread extends Thread {
    public mainThread(Runnable runnable){
        new Thread(runnable).start();
    }
}
