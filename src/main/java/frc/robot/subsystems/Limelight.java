package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    //INIT LIMELIGHT AND NT ENTRIES(VALUES)
    //use .getEntry("entryName")
    private NetworkTable limelight; 
    private NetworkTableEntry camModeEntry; //camMode | 0 - tracking; 1 - driver mode 
    private NetworkTableEntry ledModeEntry; //ledMode | 0 - pipeline sets LED; 1 - force off; 3 - force on
    private NetworkTableEntry checkTargetEntry; //tv | 0 - no valid target; 1 - valid target(s) 
    private NetworkTableEntry xOffsetEntry; //tx | horizontal offset from middle to target 
    private NetworkTableEntry pipelineEntry; //pipeline | pipelines 0-9

    //CONSTRUCTOR 
    public Limelight() {
        limelight = null; 
        camModeEntry = null; 
        ledModeEntry = null; 
        checkTargetEntry = null; 
        xOffsetEntry = null; 
        pipelineEntry = null; 
    }

    /* * * GET LIMELIGHT TABLE * * */

    //get limelight table from networkTables
    private NetworkTable getLimelight() {
        if (limelight == null) {
            limelight = NetworkTableInstance.getDefault().getTable("limelight"); 
        }

        return limelight; 
    }

    /* * * GET ENTRIES * * */
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variableName>");
    //getLimelight().getEntry("<variableName>");

    //get cam mode entry 
    private NetworkTableEntry getCamModeEntry() {
        if (limelight == null) {
            camModeEntry = null; 
        } else {
            camModeEntry = getLimelight().getEntry("camMode"); 
        }
        return camModeEntry; 
    }

    //get led mode entry 
    private NetworkTableEntry getLedModeEntry() {
        if (limelight == null) {
            ledModeEntry = null; 
        } else {
            ledModeEntry = getLimelight().getEntry("ledMode");  
        }
        return ledModeEntry; 
    }

    //get check target entry 
    private NetworkTableEntry getTargetEntry() {
        if (limelight == null) {
            checkTargetEntry = null; 
        } else {
            checkTargetEntry = getLimelight().getEntry("tv"); 
        }
        return checkTargetEntry; 
    }

    //get x offset entry 
    private NetworkTableEntry getxOffsetEntry() {
        if (limelight == null) {
            xOffsetEntry = null; 
        } else {
            xOffsetEntry = getLimelight().getEntry("tx"); 
        }
        return xOffsetEntry; 
    }

    //gets the current pipeline 
    //untested 
    private NetworkTableEntry getPipelineEntry() {
        if (limelight == null) {
            pipelineEntry = null; 
        } else {
            pipelineEntry = getLimelight().getEntry("getPipe"); 
        }
        return pipelineEntry; 
    }

    /* * * GET VALUES FROM ENTRIES * * */
    //getEntry().getDouble(0);

    private double getXOffset() {
        return getxOffsetEntry().getDouble(0);
    }

    private double getPipeline() {
        return getPipelineEntry().getDouble(0);
    }

    private boolean checkTarget() {
        return getTargetEntry().getDouble(0) == 1; 
    }

    /* * * SET ENTRIES * * */

    //set to driving mode 
    private void setDrivingMode() {
        if (getCamModeEntry() != null && getLedModeEntry() != null) {
            getCamModeEntry().setDouble(1); //set cam driver mode 
            getLedModeEntry().setDouble(1); //turn led off 
        }
    }

    //set to tracking mode 
    private void setTrackingMode() {
        if (getCamModeEntry() != null && getLedModeEntry() != null) {
            getCamModeEntry().setDouble(0); //set cam to tracking mode 
            getLedModeEntry().setDouble(3); //turn led on 
        }
    }

    private void setPipeline(double pipeline) {
        getPipelineEntry().setDouble(pipeline); //set to given pipeline 
    }

}
