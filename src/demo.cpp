/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

/*
  Documented C++ sample code of monocular visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <numeric>
#include <stdint.h>
#include <chrono>

#include <viso_mono.h>
#include <viso_stereo.h>
#include <png++/png.hpp>

using namespace std;

int main (int argc, char** argv) {

  // we need the path name to 2010_03_09_drive_0019 as input argument
  if (argc<2) {
    cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
    return 1;
  }

  // sequence directory
  string dir = argv[1];
  double mono_scale = 1.0;
  if (argc >= 3) {
    mono_scale = atof(argv[2]);
  }


  // set most important visual odometry parameters
  // for a full parameter list, look at: viso_stereo.h, viso_mono.h
  VisualOdometryMono::parameters mono_param;
  VisualOdometryStereo::parameters stereo_param;

  // monocular calibration parameters for sequence 2010_03_09_drive_0019
  mono_param.calib.f          = 645.24; // focal length in pixels
  mono_param.calib.cu         = 635.96; // principal point (u-coordinate) in pixels
  mono_param.calib.cv         = 194.13; // principal point (v-coordinate) in pixels
  // mono_param.height              = 0.6; // height above ground in meters
  // mono_param.pitch             = -0.08; // camera pitch
  mono_param.height              = 1.7; // height above ground in meters for DynSLAM test
  mono_param.pitch             = -0.03; // camera pitch
  mono_param.scale_factor = mono_scale; // Scale for translational movement

  // stereo calibration parameters for sequence 2010_03_09_drive_0019
  stereo_param.calib.f  = 645.24; // focal length in pixels
  stereo_param.calib.cu = 635.96; // principal point (u-coordinate) in pixels
  stereo_param.calib.cv = 194.13; // principal point (v-coordinate) in pixels
  stereo_param.base     = 0.5707; // baseline in meters

  // init monocular visual odometry
  VisualOdometryMono mono_viso(mono_param);

  // init stereo visual odometry
  VisualOdometryStereo stereo_viso(stereo_param);

  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix mono_pose = Matrix::eye(4);
  Matrix stereo_pose = Matrix::eye(4);

  vector<double> matrix_diffs;

  vector<double> mono_times;
  vector<double> mono_matches;
  vector<double> mono_inliers;
  int mono_num_failures = 0;

  vector<double> stereo_times;
  vector<double> stereo_matches;
  vector<double> stereo_inliers;
  int stereo_num_failures = 0;

  ofstream monocular_results;
  monocular_results.open("mono_results.txt");
  ofstream stereo_results;
  stereo_results.open("stereo_results.txt");

  // loop through all frames i=0:372
  // for (int32_t i=0; i<372; i++) {
  for (int32_t i=0; i<100; i++) {

    // input file names
    char base_name[256]; sprintf(base_name,"%06d.png",i);
    // string left_img_file_name  = dir + "/I1_" + base_name;
    // string right_img_file_name = dir + "/I2_" + base_name;
    string left_img_file_name  = dir + "/image_0/" + base_name;
    string right_img_file_name = dir + "/image_1/" + base_name;

    bool stereo_succeeded = false;
    bool mono_succeeded = false;

    // catch image read/write errors here
    try {

      // load left and right input image
      png::image< png::gray_pixel > left_img(left_img_file_name);
      png::image< png::gray_pixel > right_img(right_img_file_name);

      // image dimensions
      int32_t width  = left_img.get_width();
      int32_t height = left_img.get_height();

      // convert input images to uint8_t buffer
      uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      int32_t k=0;
      for (int32_t v=0; v<height; v++) {
        for (int32_t u=0; u<width; u++) {
          left_img_data[k]  = left_img.get_pixel(u,v);
          right_img_data[k] = right_img.get_pixel(u,v);
          k++;
        }
      }

      cout << "Frame " << i << endl;

      // MONOCULAR MOTION ESTIMATION
      // status
      // cout << "Mono Processing: Frame: " << i << endl;

      // compute monocular visual odometry
      int32_t dims[] = {width,height,width};
      auto start_time = chrono::high_resolution_clock::now();
      if (mono_viso.process(left_img_data,dims)) {
        auto end_time = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed_time = end_time - start_time;
        mono_times.push_back(elapsed_time.count());

        // on success, update current pose
        mono_pose = mono_pose * Matrix::inv(mono_viso.getMotion());
        monocular_results << mono_pose << endl << endl;

        // output some statistics
        double num_matches = mono_viso.getNumberOfMatches();
        mono_matches.push_back(num_matches);
        double num_inliers = mono_viso.getNumberOfInliers();
        mono_inliers.push_back(100*num_inliers/num_matches);
        // cout << ", Matches: " << num_matches;
        // cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
        // cout << pose << endl << endl;

        mono_succeeded = true;

      } else {
        cout << " ... mono failed!" << endl;
        mono_num_failures++;
      }

      // STEREO MOTION ESTIMATION
      // status
      // cout << "Stereo Processing: Frame: " << i;

      // compute visual odometry
      start_time = chrono::high_resolution_clock::now();
      if (stereo_viso.process(left_img_data,right_img_data,dims)) {
        auto end_time = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed_time = end_time - start_time;
        stereo_times.push_back(elapsed_time.count());

        // on success, update current pose
        stereo_pose = stereo_pose * Matrix::inv(stereo_viso.getMotion());
        stereo_results << stereo_pose << endl << endl;

        // output some statistics
        double num_matches = stereo_viso.getNumberOfMatches();
        stereo_matches.push_back(num_matches);
        double num_inliers = stereo_viso.getNumberOfInliers();
        stereo_inliers.push_back(100*num_inliers/num_matches);
        // cout << ", Matches: " << num_matches;
        // cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
        // cout << pose << endl << endl;

        stereo_succeeded = true;

      } else {
        cout << " ... stereo failed!" << endl;
        stereo_num_failures++;
      }
      // release uint8_t buffers
      free(left_img_data);
      free(right_img_data);

      if (stereo_succeeded && mono_succeeded) {
        double matrix_diff = (mono_pose-stereo_pose).l2norm();
        cout << "Difference between mono and stereo: " << matrix_diff << endl;
        matrix_diffs.push_back(matrix_diff);
      }

    // catch image read errors here
    } catch (...) {
      cerr << "ERROR: Couldn't read input files!" << endl;
      return 1;
    }
  }

  // output
  double average_matrix_diff = accumulate(matrix_diffs.begin(), matrix_diffs.end(), 0.0)/matrix_diffs.size();
  cout << "Average matrix difference: " << average_matrix_diff << endl;
  // double average_processing_time = accumulate(times.begin(), times.end(), 0.0)/times.size();
  // double average_inlier_percentage = accumulate(inliers.begin(), inliers.end(), 0.0)/inliers.size();
  // cout << "Average processing time: " << average_processing_time << endl;
  // cout << "Average Inlier percentage: " << average_inlier_percentage << endl;
  // cout << "Number of failures: " << num_failures << endl;

  monocular_results.close();
  stereo_results.close();
  cout << "Demo complete! Exiting ..." << endl;

  // exit

  return 0;
}
