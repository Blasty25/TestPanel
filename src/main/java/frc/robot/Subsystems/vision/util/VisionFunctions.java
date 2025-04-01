package frc.robot.Subsystems.vision.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.vision.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionFunctions {
    /**
     * Estimates the standard deviations that should be used for a pose given the
     * distance and number of cameras that can see the target.
     * 
     * @param camera
     * @param estimatedPose
     * @param poseEstimator
     */
    public static Matrix<N3, N1> getStdDevs(PhotonPipelineResult result, Pose3d estimatedPose, AprilTagFieldLayout layout) {
        List<PhotonTrackedTarget> targets = result.targets;
        int numTags = 0;
        double dist = 0;
        
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = layout.getTagPose(target.getFiducialId());

            if (tagPose.isEmpty()) continue;

            numTags++;

            // Looks weird, but it is only used if there is one tag, so it doesn't need to
            // be averaged.
            dist += tagPose.get().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0) return VisionConstants.singleTagStdDevs;
        
        if (numTags > 1) return VisionConstants.multiTagStdDevs;

        if (dist > 4) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        return VisionConstants.singleTagStdDevs.times(1 + (dist * dist / 30));
    }
}