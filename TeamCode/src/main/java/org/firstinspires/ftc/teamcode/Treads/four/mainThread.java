package org.firstinspires.ftc.teamcode.Treads.four;

class Single implements Runnable {
    @Override
    public void run() {
    }
}

class Test
{
    public static void mainThread(){
        Thread test = new Thread(new Single());
        test.start();

    }
}


