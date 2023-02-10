package com.team2502.demo2022.subsystems;

import com.team2502.demo2022.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightVisionSubsystem extends SubsystemBase {

    //Defines all Objects
    private final NetworkTable Limelight;
    private final NetworkTable Dashboard;

    private double targetX;
    private double targetY;
    private double targetA;

    public LimelightVisionSubsystem() {
        //sets Object Constants
        Limelight = NetworkTableInstance.getDefault().getTable(Constants.Subsystems.Vision.LIMELIGHT_NETWORK_TABLE);
        Dashboard = NetworkTableInstance.getDefault().getTable("smartdashboard");
    }

    @Override
    //runs often the retrieve the newest values from the Limelight
    public void periodic() {
        NetworkTableEntry TARGET_X = Limelight.getEntry("x");
        NetworkTableEntry TARGET_Y = Limelight.getEntry("y");
        NetworkTableEntry AREA_51 = Limelight.getEntry("area");

        //sets the doubles to the Limelight's values & sets to zero if no targets found
        targetX = TARGET_X.getDouble(0);
        targetY = TARGET_Y.getDouble(0);
        targetA = AREA_51.getDouble(0);
    }
    //returns the X offset
    public double getX(){
        return targetX;
    }
    //returns the Y offset
    public double getY(){
        return targetY;
    }
    //returns the target Area
    public double getAREA(){
        return targetA;
    }
    //returns true if there is a target, false is one is not found
    public boolean targot(){
        return (targetA != 0);
    }
}

