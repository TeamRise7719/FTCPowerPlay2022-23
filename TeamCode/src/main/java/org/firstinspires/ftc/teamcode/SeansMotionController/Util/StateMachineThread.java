package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 1/15/23.
 */
public class StateMachineThread extends Thread {

    public String state;

    public StateMachineThread(String state){
        this.state = state;
    }

    @Override
    public void run() {

    }

    public void setState(String state) {
        this.state = state;
    }

}
