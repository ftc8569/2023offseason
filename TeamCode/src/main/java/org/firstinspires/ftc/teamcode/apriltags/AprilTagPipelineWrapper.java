package org.firstinspires.ftc.teamcode.apriltags;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.apriltags.ConeNumber;
import org.firstinspires.ftc.teamcode.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagPipelineWrapper extends SubsystemBase {

    private final AprilTagDetectionPipeline pipeline;
    private final Telemetry t;
    private final DrivetrainSubsystem dt;
    public AprilTagPipelineWrapper(DrivetrainSubsystem dt,FtcDashboard dash, Telemetry t){
        this.dt = dt;
        this.t = t;
        this.pipeline = new AprilTagDetectionPipeline();
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