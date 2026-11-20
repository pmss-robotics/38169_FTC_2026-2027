package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;


//LIMELIGHTS read the Apriltags themselves, so this class only starts it and reads the latest result.

public class LimelightSubsystem {

    private final Limelight3A limelight;
    private LLResult latestResult;

    /**
     * @param hardwareMap   the opmode's hardware map
     * @param deviceName    name of the Limelight in the Control Hub configuration
     * @param pipelineIndex Limelight pipeline that has AprilTag detection enabled
     */

    public LimelightSubsystem(HardwareMap hardwareMap, String deviceName, int pipelineIndex) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.pipelineSwitch(pipelineIndex);
    }


    //FUNCTIONS FOR POLLING LOOP

    /** Start polling (getting results) . getLatestResult() returns null until this is called. */
    public void start() {
        limelight.start();
    }

    /** Stop polling. */
    public void stop() {
        limelight.stop();
    }

    /** Call once per loop to refresh the data. */
    public void update() {
        latestResult = limelight.getLatestResult();
    }



    //Does limelight have a valid result?
    public boolean hasValidResult() {
        return latestResult != null && latestResult.isValid();
    }

    //List all April tags seen in frame, or return empty list if none
    public List<LLResultTypes.FiducialResult> getTags() {
        if (!hasValidResult()) {
            return new ArrayList<>();
        }
        return latestResult.getFiducialResults();
    }

    //GET LATEST RAW RESULT, OR NULL RESULT
    public LLResult getResult() {
        return latestResult;
    }






    // STATUS GETTERS
    public com.qualcomm.hardware.limelightvision.LLStatus getStatus() {
        return limelight.getStatus();
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public boolean isRunning() {
        return limelight.isRunning();
    }
}
