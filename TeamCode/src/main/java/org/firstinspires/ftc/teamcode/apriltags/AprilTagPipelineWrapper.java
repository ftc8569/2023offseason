package org.firstinspires.ftc.teamcode.apriltags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltags.ConeNumber;
import org.firstinspires.ftc.teamcode.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.AutoDrive;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagPipelineWrapper extends SubsystemBase {

    private final AprilTagDetectionPipeline pipeline;
    private final Telemetry t;
    private final AutoDrive dt;
    public AprilTagPipelineWrapper(AutoDrive dt,FtcDashboard dash, Telemetry t){
        this.dt = dt;
        this.t = t;
        this.pipeline = new AprilTagDetectionPipeline(dash);
    }

    public AprilTagDetectionPipeline getPipeline(){
        return pipeline;
    }

    public ArrayList<AprilTagDetection> getSleeveDetections(){
        return pipeline.getLatestDetections();
    }

    @Override
    public void periodic(){

    }

}