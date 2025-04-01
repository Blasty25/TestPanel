package frc.robot.Subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.vision.util.VisionResult;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
    @AutoLog
    public class CameraIOInputs {
        String cameraName;
        VisionResult[] unreadResults;
        boolean isActive;
    }

    /**
     * Updates a set of IO inputs with current values.
     * 
     * @param inputs The inputs to update.
     */
    public void updateInputs();

    /** Gets the name of the camera. */
    public String getName();

    /** Gets all of the unread vision results from each camera. */
    public ArrayList<VisionResult> getUnreadResults();

    /** Gets whether or not the camera is connected to the robot. */
    public boolean isConnected();

    /**
     * Updates the cameras with the current pose of the robot.
     * 
     * @param pose The pose of the robot.
     */
    public void setRobotPose(Pose2d robotPose);
}