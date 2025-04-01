package frc.robot.Subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Subsystems.vision.util.VisionFunctions;
import frc.robot.Subsystems.vision.util.VisionResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class CameraIOSim implements CameraIO {
    private VisionSystemSim visionSim;

    private PhotonCamera camera;
    private PhotonCameraSim simCamera;
    private PhotonPoseEstimator poseEstimator;

    private CameraIOInputsAutoLogged inputs;

    public CameraIOSim(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);

        SimCameraProperties camProperties = new SimCameraProperties();
        camProperties.setCalibration(320, 320, Rotation2d.fromDegrees(70));
        camProperties.setFPS(90);

        simCamera = new PhotonCameraSim(camera);

        visionSim = new VisionSystemSim("VisionSim");
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
        visionSim.addCamera(simCamera, robotToCamera);

        poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

        inputs = new CameraIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
        inputs.cameraName = getName();
        inputs.isActive = isConnected();
        
        // Convert the ArrayList<VisionResult> to VisionResult[] array
        inputs.unreadResults = getUnreadResults().toArray(new VisionResult[0]);
    }

    @Override
    public String getName() {
        return camera.getName();
    }

    @Override
    public ArrayList<VisionResult> getUnreadResults() {
        List<PhotonPipelineResult> results = simCamera.getCamera().getAllUnreadResults();

        ArrayList<VisionResult> visionResults = new ArrayList<>(results.size());

        for (int i = 0; i < results.size(); i++) {
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(results.get(i));

            if (estimatedPose.isEmpty()) continue;

            visionResults.add(new VisionResult(
                    estimatedPose.get().estimatedPose,
                    estimatedPose.get().timestampSeconds,
                    VisionFunctions.getStdDevs(results.get(i), estimatedPose.get().estimatedPose,
                            poseEstimator.getFieldTags())));
        }

        return visionResults;
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        visionSim.update(pose);
    }
}
