package org.firstinspires.ftc.teamcode.Components.RFModules.System;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

/**
 * Warren Zhou
 * 8/23
 * Data tpe for individual event
 */
public class QueueElement {
    //place in line
    private final int queuePos;
    //after which item is done u can run
    public int startCondition;
    //if event is asynchronous(if should be sequential or ignore)
    private final boolean asynchronous;
    //if started
    private boolean started = false;
    //if done
    private boolean isDone = false;
    //For calculating delay, 1000 is just a placeholder that should never be reached normally
    private double readyTime = 1000;
    //should it wait for everything else to finish
    private final boolean mustFinish;
    //should it wait for everything else to finish
    private final boolean shouldFinish;
    //is it an optional event
    private final boolean isOptional;
    //is it skippable right now
    private boolean option = false;

    private boolean isExecuted = false;

    /**
     * Initialize QueueElement
     * @param p_queuePos index of the queue element
     * @param p_asynchronous should the event be asynchronous
     * @param p_startCondition when should the event start
     * @param p_mustFinish does the event need to finish
     */
    public QueueElement(int p_queuePos, boolean p_asynchronous, int p_startCondition, boolean p_mustFinish) {
        this(p_queuePos, p_asynchronous, p_startCondition,p_mustFinish,false,false);
    }

    /**
     * Initialize QueueElement
     * @param p_queuePos index of the queue element
     * @param p_asynchronous should the event be asynchronous
     * @param p_startCondition when should the event start
     * @param p_mustFinish does the event need to finish
     * @param p_shouldFinish does the event need to finish
     * @param p_isOptional is the event optional
     */
    public QueueElement(int p_queuePos, boolean p_asynchronous, int p_startCondition, boolean p_mustFinish, boolean p_shouldFinish, boolean p_isOptional) {
        queuePos = p_queuePos;
        asynchronous = p_asynchronous;
        startCondition = p_startCondition;
        mustFinish = p_mustFinish;
        shouldFinish = p_shouldFinish;
        isOptional = p_isOptional;
    }

    /**calculate if Ready
     *
     * @param p_currentEvent what event is queuer currently queueing
     * @param p_extraCondition what extra conditions need to be satisfied before running this event
     * @return boolean for if the function should the function run
     */
    public boolean isReady(int p_currentEvent, boolean p_extraCondition) {
        //is it this elements turn to run
        if (p_currentEvent >= startCondition && !isDone) {
            // update when start for delay logic
            if (readyTime > time) {
                readyTime = time;
                logger.log("/RobotLogs/GeneralRobot", queuePos + "readyTime =" + readyTime +"optional"+isOptional);
            }
            //consider extraw condition(if it exists)
            //                logger.log("/RobotLogs/GeneralRobot", queuePos + "readyTime =" + readyTime);
            return p_extraCondition;
        } else {
            return false;
        }
    }

    /**
     * get & set functions
     */
    public boolean isMustFinish() {
        return mustFinish;
    }

    public boolean isDone() {
        return isDone;
    }

    public void setDone(boolean p_isDone) {
        isDone = p_isDone;
    }

    public void setStarted(boolean p_started) {
        started = p_started;
    }

    public boolean isStarted() {
        return started;
    }

    public boolean isAsynchronous() {
        return asynchronous;
    }

    public void setStartCondition(int p_condition){
        startCondition=p_condition;
    }

    public boolean isOptional(){return isOptional;}

    public boolean getSkipOption(){return option;}

    public void setSkipOption(boolean p_option){option = p_option;}

    public double getReadyTime() {
        return readyTime;
    }

    public boolean isShouldFinish() {
        return shouldFinish;
    }

    public void setExecuted(boolean p_executed){isExecuted=p_executed;}
    public boolean isExecuted(){return isExecuted;}
}
