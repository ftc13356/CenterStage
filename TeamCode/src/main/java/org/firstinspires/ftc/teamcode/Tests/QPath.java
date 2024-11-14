package org.firstinspires.ftc.teamcode.Tests;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;

import java.util.List;

public class QPath extends TrajectorySequence {
    boolean isDone = false;
    public QPath(List<SequenceSegment> sequenceList) {
        super(sequenceList);
    }

    public void setDone(boolean isDone){
        this.isDone = isDone;
    }

}
