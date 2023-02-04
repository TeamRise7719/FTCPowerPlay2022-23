package org.firstinspires.ftc.teamcode.SeansMotionController.Util;

/**
 * Created by Sean Cardosi on 1/15/23.
 */
public class StateChange extends HeadingControlledWaypoint {

    public StateMachineThread thread;
    public String state;

    public StateChange(StateMachineThread thread, String state) {
        super(0, 0, 0,0, false, 0);
        this.thread = thread;
        this.state = state;
    }
}
