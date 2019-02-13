/**
* This file is part of Ewok.
*
* Copyright 2017 Vladyslav Usenko, Technical University of Munich.
* Developed by Vladyslav Usenko <vlad dot usenko at tum dot de>,
* for more information see <http://vision.in.tum.de/research/robotvision/replanning>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Ewok is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Ewok is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Ewok. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef EWOK_POLY_SPLINE_INCLUDE_EWOK_POLYNOMIAL_TRAJECTORY_3D_H_
#define EWOK_POLY_SPLINE_INCLUDE_EWOK_POLYNOMIAL_TRAJECTORY_3D_H_

#include <ewok/polynomial_segment_3d.h>

#include <visualization_msgs/MarkerArray.h>

namespace ewok {

template<int _N, typename _Scalar = double>
class PolynomialTrajectory3D {
 public:

  typedef Polynomial <_N, _Scalar> Pol;
  typedef PolynomialSegment3D <_N, _Scalar> PolSeg;
  typedef Eigen::Matrix<_Scalar, 3, 1> Vector3;

  typedef typename PolynomialSegment3D<_N, _Scalar>::Ptr PolSegPtr;

  typedef std::shared_ptr <PolynomialTrajectory3D<_N, _Scalar>> Ptr;

  PolynomialTrajectory3D() {
      _cumulative_time.push_back(0);
  }

  void addSegment(PolSegPtr &seg) {
      _cumulative_time.push_back(_cumulative_time.back() + seg->duration());
      _segments.push_back(seg);
  }

  _Scalar duration() const {
      return _cumulative_time.back();
  }

  int numSegments() const {
      return _segments.size();
  }

  Vector3 evaluate(const _Scalar t, int derivative = 0) const {
      _Scalar lt;
      size_t seg_num;

      findSegmentNumAndLocalTime(t, lt, seg_num);
      return _segments[seg_num]->evaluate(lt, derivative);
  }

  void getRemovedSegmentTraj(visualization_msgs::MarkerArray &traj_marker_array,
                              const double start_seg_time,
                              const size_t start_seg_num,
                              const double end_seg_time,
                              const size_t end_seg_num,
                              const std::string &ns,
                              const Eigen::Vector3d &color,
                              _Scalar dt = 0.1) {
      //traj_marker_array.markers.resize(_segments.size());

      _Scalar start_time, end_time;
      int counter = 1;
      for (int i = start_seg_num; i < _segments.size(); i++) {
        start_time = 0;
        _Scalar seg_end_dur = _segments[i]->duration();
        end_time = seg_end_dur;
        if((i == start_seg_num) && (i == end_seg_num)) {
          /*
          start_time = 0;
          end_time = start_seg_time;
          _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                               start_time,
                                               end_time,
                                               ns,
                                               counter,
                                               color,
                                               dt);
          */
          start_time = end_seg_time;
          end_time = seg_end_dur;
          //Append new segment passed externally
          for(int jj = 0; jj < traj_marker_array.markers[0].points.size(); jj++)
          {
            traj_marker_array.markers[counter].points.push_back(traj_marker_array.markers[0].points[jj]);
          }
          _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                               start_time,
                                               end_time,
                                               ns,
                                               counter,
                                               color,
                                               dt);
          counter++;
          continue;
        }
        else {
          if(i == start_seg_num) {
            /*
            end_time = start_seg_time;
            _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                                 start_time,
                                                 end_time,
                                                 ns,
                                                 counter,
                                                 color,
                                                 dt);
            */
            for(int jj = 0; jj < traj_marker_array.markers[0].points.size(); jj++)
            {
              traj_marker_array.markers[counter].points.push_back(traj_marker_array.markers[0].points[jj]);
            }
            counter++;
            continue;
          }
          if(i > start_seg_num && i < end_seg_num)
            continue;
          if(i == end_seg_num) {
            start_time = end_seg_time;
            _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                                 start_time,
                                                 end_time,
                                                 ns,
                                                 counter,
                                                 color,
                                                 dt);
            counter++;
            continue;
          }
        }
        //Default case
        start_time = 0;
        end_time = seg_end_dur;
        _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                             start_time,
                                             end_time,
                                             ns,
                                             counter,
                                             color,
                                             dt);
        counter++;
      }

      //TODO remove unnecessary segments
      /*
      if(counter < _segments.size())
      {
        traj_marker_array.markers.erase(traj_marker_array.markers.begin() + counter, traj_marker_array.markers.end());
      }*/
  }

/*
  void getRemovedSegmentTraj(visualization_msgs::MarkerArray &traj_marker_array,
                              const double start_seg_time,
                              const size_t start_seg_num,
                              const double end_seg_time,
                              const size_t end_seg_num,
                              const std::string &ns,
                              const Eigen::Vector3d &color,
                              _Scalar dt = 0.1) {
      //traj_marker_array.markers.resize(_segments.size());

      _Scalar start_time, end_time;
      int counter = 1;
      for (int i = 0; i < _segments.size(); i++) {
        start_time = 0;
        _Scalar seg_end_dur = _segments[i]->duration();
        end_time = seg_end_dur;
        if((i == start_seg_num) && (i == end_seg_num)) {
          start_time = 0;
          end_time = start_seg_time;
          _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                               start_time,
                                               end_time,
                                               ns,
                                               counter,
                                               color,
                                               dt);
          start_time = end_seg_time;
          end_time = seg_end_dur;
          //Append new segment passed externally
          for(int jj = 0; jj < traj_marker_array.markers[0].points.size(); jj++)
          {
            traj_marker_array.markers[counter].points.push_back(traj_marker_array.markers[0].points[jj]);
          }
          _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                               start_time,
                                               end_time,
                                               ns,
                                               counter,
                                               color,
                                               dt);
          counter++;
          continue;
        }
        else {
          if(i == start_seg_num) {
            end_time = start_seg_time;
            _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                                 start_time,
                                                 end_time,
                                                 ns,
                                                 counter,
                                                 color,
                                                 dt);
            for(int jj = 0; jj < traj_marker_array.markers[0].points.size(); jj++)
            {
              traj_marker_array.markers[counter].points.push_back(traj_marker_array.markers[0].points[jj]);
            }
            counter++;
            continue;
          }
          if(i > start_seg_num && i < end_seg_num)
            continue;
          if(i == end_seg_num) {
            start_time = end_seg_time;
            _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                                 start_time,
                                                 end_time,
                                                 ns,
                                                 counter,
                                                 color,
                                                 dt);
            counter++;
            continue;
          }
        }
        //Default case
        start_time = 0;
        end_time = seg_end_dur;
        _segments[i]->getRemovedSegmentTraj(traj_marker_array.markers[counter],
                                             start_time,
                                             end_time,
                                             ns,
                                             counter,
                                             color,
                                             dt);
        counter++;
      }

  }
*/

  void getVisualizationMarkerArray(visualization_msgs::MarkerArray &traj_marker_array,
                                   const std::string &ns,
                                   const Eigen::Vector3d &color,
                                   _Scalar dt = 0.1) {
      traj_marker_array.markers.resize(_segments.size());

      for (int i = 0; i < _segments.size(); i++) {
          _segments[i]->getVisualizationMarker(traj_marker_array.markers[i],
                                               ns,
                                               i,
                                               color,
                                               dt);
      }
  }

  void findSegmentNumAndLocalTime(const _Scalar t,
                                  _Scalar &local_time,
                                  size_t &segment_number) const {
      if (t > _cumulative_time.back()) {
          segment_number = _segments.size() - 1;
          local_time = t - _cumulative_time[segment_number];
      } else if (t <= 0) {
          local_time = 0;
          segment_number = 0;
      } else {
          segment_number = 0;
          while (_cumulative_time[segment_number] < t) {
              segment_number++;
          }
          segment_number--;
          //std::cerr << "In while loop segment_number: " << segment_number << std::endl;
          local_time = t - _cumulative_time[segment_number];
      }
  }

  std::vector <_Scalar> _cumulative_time;
  std::vector <PolSegPtr> _segments;

};

}

#endif //  EWOK_POLY_SPLINE_INCLUDE_EWOK_POLYNOMIAL_TRAJECTORY_3D_H_
