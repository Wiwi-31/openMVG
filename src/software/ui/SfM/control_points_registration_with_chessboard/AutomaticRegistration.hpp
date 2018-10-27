// Copyright (c) 2018 William GELARD.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <iostream>
#include <algorithm>
#include <clocale>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/multiview/triangulation_nview.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"
#include "openMVG/geometry/rigid_transformation3D_srt.hpp"
#include "openMVG/geometry/Similarity3.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_transform.hpp"

#include "openMVG/stl/stl.hpp"

#include "document.hpp"


#pragma once

namespace control_point_with_chessboard
{

  /// QT Interface to edit Landmarks GCP data:
  /// Allow to edit X,Y,Z Ground Control Point coordinates
  /// Allow to delete GCP
  class AutomaticRegistration
  {
    public:

      enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

      AutomaticRegistration(const std::string sfm_data_fileName,
                            const std::string settings_file);

      /// Update control points X,Y,Z data (if valid datum is provided)
      void update_control_points (openMVG::sfm::Landmarks &control_points);

      /// Save the SfM_Data scene
      void saveProject();

      /// Open a SfM_Data scene
      void openProject();

      /// Find chessboard corner
      bool findChessboard(const std::string &s_filename, IndexT id_view);

      /// Perform the registration of the SfM_Data scene to the GCP
      void registerProject();


    private:

      // Data file
      std::string m_sfm_data_filename;
      std::string m_settings_file;
      Document m_doc;


      // Chessboard information
      Pattern m_patternType;
      cv::Size m_boardSize;
      float m_squareSize;
  };

} // namespace control_point_with_chessboard
