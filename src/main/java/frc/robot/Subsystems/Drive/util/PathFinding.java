// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class PathFinding {

    PathPlannerPath path;
    Pose2d targetPose;

    PathConstraints constraints;

    public PathFinding() {

        try {
            path = PathPlannerPath.fromPathFile("test1");
            targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180)); // Change later
        } catch (Exception e) {
            e.printStackTrace();
        }

        constraints = new PathConstraints(3.0,
                4.0,
                Units.degreesToRadians(540),
                Units.degreesToRadians(720));

    }

    public Command pathfindingCommand(){
        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }

    public Command pathFindingPose(){
        return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    }
}
