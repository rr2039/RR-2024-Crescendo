// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionUtils extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPipelineResult latest_result;

  /** Creates a new PhotonVisionUtils. */
  public PhotonVisionUtils(String camName) {
    camera = new PhotonCamera(camName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    latest_result = camera.getLatestResult();
  }
}
