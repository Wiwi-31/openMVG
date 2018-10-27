// Copyright (c) 2018 William GELARD.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "AutomaticRegistration.hpp"

namespace control_point_with_chessboard {

  using namespace openMVG;
  using namespace openMVG::cameras;
  using namespace openMVG::sfm;

  AutomaticRegistration::AutomaticRegistration(const std::string sfm_data_fileName,
                                               const std::string settings_file)
    : m_sfm_data_filename(sfm_data_fileName),
      m_settings_file(settings_file)
  {
  }

  void AutomaticRegistration::openProject()
  {
    if (m_doc.loadData(m_sfm_data_filename))
    {

      // Reset control points
      m_doc._sfm_data.control_points = Landmarks();

      // Load the pictures
      std::vector<IndexT> view_ids;
      view_ids.reserve(m_doc._sfm_data.GetViews().size());
      std::transform(m_doc._sfm_data.GetViews().begin(), m_doc._sfm_data.GetViews().end(),
                     std::back_inserter(view_ids), stl::RetrieveKey());
      std::sort(view_ids.begin(), view_ids.end());

      std::cout << "Nb views: " << m_doc._sfm_data.GetViews().size() << std::endl;
      int nbPerformed = 0;

      // Load chessboard informations
      if (m_settings_file.empty())
      {
        // Set default config: 4x11 asymmetric circles grid
        m_patternType=ASYMMETRIC_CIRCLES_GRID;

        m_boardSize.width = 4;
        m_boardSize.height = 11;
        m_squareSize = 0.02;
      }
      else
      {
        /// TODO
        // Load config from settings_file

        //        cv::Settings s;
        //        cv::FileStorage fs(m_settings_file, FileStorage::READ); // Read the settings

        //        if (!fs.isOpened()){
        //          std::cout << "Could not open the configuration file: \"" << m_settings_file << "\"" << std::endl;
        //          return -1;
        //        }
        //        fs["Settings"] >> s;
        //        fs.release(); // close Settings file

        //        if (!s.goodInput){
        //          std::cout << "Invalid input detected. Application stopping. " << std::endl;
        //          return -1;
        //        }

        m_patternType=ASYMMETRIC_CIRCLES_GRID;

        m_boardSize.width = 4;
        m_boardSize.height = 11;
        m_squareSize = 0.02;
      }

      for (int i=0, s=m_doc._sfm_data.GetViews().size(); i<s; ++i)
      {
        const View * view = m_doc._sfm_data.GetViews().at(i).get();
        if (m_doc._sfm_data.IsPoseAndIntrinsicDefined(view))
        {
          const std::string sView = stlplus::create_filespec(m_doc._sfm_data.s_root_path, view->s_Img_path);
          IndexT current_view_id = view->id_view;
          
          if (findChessboard(sView, current_view_id))
          {
            nbPerformed++;
          }

          if (nbPerformed >= 2)
            break;
        }
      }

      // Update data file
      update_control_points(m_doc._sfm_data.control_points);

      // Register project
      registerProject();

      // Save file
      std::setlocale(LC_ALL, "C");
      if (!m_doc.saveData(m_sfm_data_filename))
      {
        std::cout << "Error saving file!\n";
      }
    }
  }

  /// Update control points X,Y,Z data (if valid datum is provided)
  void AutomaticRegistration::update_control_points(Landmarks & control_points)
  {
    int height = m_boardSize.height;
    int width = m_boardSize.width;
    double squareSize = m_squareSize;

    // Get back the (updated) control points coordinates
    //    for (int i = 0; i < (height*width); ++i)
    //    {
    //      const double
    //          valueX = (i%width)*squareSize,
    //          valueY = (i/width)*squareSize,
    //          valueZ = 0;

    //      control_points[i].X << valueX, valueY, valueZ;
    //    }

    for (int i = 0; i < (height*width); ++i)
    {
      double valX = (i%width)*squareSize*2;

      if ((i/width)%2==0)
      {
        valX += squareSize;
      }

      const double
          valueX = valX,
          valueY = (i/width)*squareSize,
          valueZ = 0;

      //      std::cout << "Pts: " << valueX << "; " << valueY << std::endl;
      control_points[i].X << valueX, valueY, valueZ;
    }
  }

  bool AutomaticRegistration::findChessboard(const std::string &s_filename, IndexT id_view)
  {
    if (m_doc._sfm_data.GetViews().empty())
    {
      return false;
    }

    std::cout << "Working on: " << s_filename;
    
    cv::Mat img = cv::imread(s_filename, CV_LOAD_IMAGE_COLOR);
    if (!img.data)
    {
      std::cout << "Could not open or find the image: " << s_filename << std::endl;
      exit(-1);
    }

    std::vector<cv::Point2f> ptvec;

    bool found;
    switch (m_patternType) // Find feature points on the input format
    {
      case CHESSBOARD:
        found = findChessboardCorners(img, m_boardSize, ptvec,
                                      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        break;
      case CIRCLES_GRID:
        found = findCirclesGrid(img, m_boardSize, ptvec);
        break;
      case ASYMMETRIC_CIRCLES_GRID:
        found = findCirclesGrid(img, m_boardSize, ptvec, cv::CALIB_CB_ASYMMETRIC_GRID);
        break;
    }


    if (found)
    {
      for (int i=0; i<ptvec.size(); ++i){
        int ind = (m_boardSize.width - 1 - i%m_boardSize.width)+m_boardSize.width*(i/m_boardSize.width);
        // Try to setup a new landmark:
        Landmark & landmark = m_doc._sfm_data.control_points[i];
        if (landmark.obs.count(id_view) == 0){
          landmark.obs[id_view] = Observation(Vec2(ptvec[ind].x, ptvec[ind].y), 0);
        }
        else
        {
          // If the control point was not existing, clear it
          if (landmark.obs.empty())
          {
            m_doc._sfm_data.control_points.erase(m_doc._sfm_data.control_points.find(i));
          }
        }
      }
    }

    std::cout << " -> " << found << std::endl;
    return found;
  }

  void AutomaticRegistration::registerProject()
  {

    if (m_doc._sfm_data.control_points.size() < 3)
    {
      std::cout << "At least 3 control points are required.\n";
      return;
    }

    // Assert that control points can be triangulated
    for (Landmarks::const_iterator iterL = m_doc._sfm_data.control_points.begin();
         iterL != m_doc._sfm_data.control_points.end(); ++iterL){
      if (iterL->second.obs.size() < 2)
      {
        std::cout << "Each control point must be defined in at least 2 pictures.\n";
        return;
      }
    }

    //---
    // registration (coarse):
    // - compute the 3D points corresponding to the control point observation for the SfM scene
    // - compute a coarse registration between the controls points & the triangulated point
    // - transform the scene according the found transformation
    //---
    std::map<IndexT, Vec3> vec_control_points, vec_triangulated;
    std::map<IndexT, double> vec_triangulation_errors;
    for (const auto & control_point_it : m_doc._sfm_data.control_points)
    {
      const Landmark & landmark = control_point_it.second;
      //Triangulate the observations:
      const Observations & obs = landmark.obs;
      std::vector<Vec3> bearing;
      std::vector<Mat34> poses;
      bearing.reserve(obs.size());
      poses.reserve(obs.size());
      for (const auto & obs_it : obs)
      {
        const View * view = m_doc._sfm_data.views.at(obs_it.first).get();
        if (!m_doc._sfm_data.IsPoseAndIntrinsicDefined(view))
          continue;
        const openMVG::cameras::IntrinsicBase * cam = m_doc._sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
        const openMVG::geometry::Pose3 pose = m_doc._sfm_data.GetPoseOrDie(view);
        const Vec2 pt = obs_it.second.x;
        bearing.emplace_back((*cam)(cam->get_ud_pixel(pt)));
        poses.emplace_back(pose.asMatrix());
      }
      const Eigen::Map<const Mat3X> bearing_matrix(bearing[0].data(), 3, bearing.size());
      Vec4 Xhomogeneous;
      if (!TriangulateNViewAlgebraic(bearing_matrix, poses, &Xhomogeneous))
      {
        std::cout << "Invalid triangulation" << std::endl;
        return;
      }
      const Vec3 X = Xhomogeneous.hnormalized();
      // Test validity of the hypothesis (front of the cameras):
      bool bChierality = true;
      int i(0);
      double reprojection_error_sum(0.0);
      for (const auto & obs_it : obs)
      {
        const View * view = m_doc._sfm_data.views.at(obs_it.first).get();
        if (!m_doc._sfm_data.IsPoseAndIntrinsicDefined(view))
          continue;

        const Pose3 pose = m_doc._sfm_data.GetPoseOrDie(view);
        bChierality &= CheiralityTest(bearing[i], pose, X);
        const openMVG::cameras::IntrinsicBase * cam = m_doc._sfm_data.GetIntrinsics().at(view->id_intrinsic).get();
        const Vec2 pt = obs_it.second.x;
        const Vec2 residual = cam->residual(pose(X), pt);
        reprojection_error_sum += residual.norm();
        ++i;
      }
      if (bChierality) // Keep the point only if it has a positive depth
      {
        vec_triangulated[control_point_it.first] = X;
        vec_control_points[control_point_it.first] = landmark.X;
        vec_triangulation_errors[control_point_it.first] = reprojection_error_sum/(double)bearing.size();
      }
      else
      {
        std::cout << "Control Point cannot be triangulated (not in front of the cameras)" << std::endl;
        return;
      }
    }

    if (vec_control_points.size() < 3)
    {
      std::cout << "Insufficient number of triangulated control points.\n";
      return;
    }

    // compute the similarity
    {
      // data conversion to appropriate container
      Mat x1(3, vec_control_points.size()),
          x2(3, vec_control_points.size());
      for (int i=0; i < vec_control_points.size(); ++i){
        x1.col(i) = vec_triangulated[i];
        x2.col(i) = vec_control_points[i];
      }

      //      std::cout
      //          << "Control points observation triangulations:\n"
      //          << x1 << std::endl << std::endl
      //          << "Control points coords:\n"
      //          << x2 << std::endl << std::endl;

      Vec3 t;
      Mat3 R;
      double S;
      if (openMVG::geometry::FindRTS(x1, x2, &S, &t, &R))
      {
        openMVG::geometry::Refine_RTS(x1,x2,&S,&t,&R);
        std::cout << "\nFound transform:\n"
                  << " scale: " << S << "\n"
                  << " rotation:\n" << R << "\n"
                  << " translation: "<< t.transpose() << std::endl;

        //--
        // Apply the found transformation as a 3D Similarity transformation matrix // S * R * X + t
        //--
        const openMVG::geometry::Similarity3 sim(geometry::Pose3(R, -R.transpose() * t/S), S);
        openMVG::sfm::ApplySimilarity(sim, m_doc._sfm_data);

        //        // Display some statistics:
        //        std::stringstream os;
        //        for (Landmarks::const_iterator iterL = m_doc._sfm_data.control_points.begin();
        //             iterL != m_doc._sfm_data.control_points.end(); ++iterL)
        //        {
        //          const IndexT CPIndex = iterL->first;
        //          // If the control point has not been used, continue...
        //          if (vec_triangulation_errors.find(CPIndex) == vec_triangulation_errors.end())
        //          {
        //            continue;
        //          }

        //          os
        //            << "CP index: " << CPIndex << "\n"
        //            << "CP triangulation error: " << vec_triangulation_errors[CPIndex] << " pixel(s)\n"
        //            << "CP registration error: "
        //            << (sim(vec_triangulated[CPIndex]) - vec_control_points[CPIndex]).norm() << " user unit(s)"<< "\n\n";
        //        }
        //        std::cout << os.str();
      }
      else
      {
        std::cout << "Registration failed. Please check your Control Points coordinates.\n";
      }
    }

    //---
    // Bundle adjustment with GCP
    //---
    {
      using namespace openMVG::sfm;
      Bundle_Adjustment_Ceres::BA_Ceres_options options;
      Bundle_Adjustment_Ceres bundle_adjustment_obj(options);
      Control_Point_Parameter control_point_opt(20.0, true);
      if (!bundle_adjustment_obj.Adjust(m_doc._sfm_data,
          Optimize_Options
          (
            cameras::Intrinsic_Parameter_Type::NONE, // Keep intrinsic constant
            Extrinsic_Parameter_Type::ADJUST_ALL, // Adjust camera motion
            Structure_Parameter_Type::ADJUST_ALL, // Adjust structure
            control_point_opt // Use GCP and weight more their observation residuals
            )
          )
          )
      {
        std::cout << "BA with GCP failed.\n";
      }
    }
  }

} // namespace control_point_GUI
