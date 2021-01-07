// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <boost/circular_buffer.hpp>
#include "imageProjection.h"

ImageProjection::ImageProjection(ros::NodeHandle& nh,
                                 Channel<ProjectionOut>& output_channel)
    : _nh(nh),
      _output_channel(output_channel)
{
  // 订阅激光点
  _sub_laser_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
      "/lidar_points", 1, &ImageProjection::cloudHandler, this);
  // 发布所有点云
  _pub_full_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
  // 发布所有点云信息
  _pub_full_info_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

  // 发布地面点
  _pub_ground_cloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
  // 发布分割点
  _pub_segmented_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
  // 发布纯分割点?
  _pub_segmented_cloud_pure =
      nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);
  // 发布分割点信息
  _pub_segmented_cloud_info =
      nh.advertise<cloud_msgs::cloud_info>("/segmented_cloud_info", 1);
  // 发布局外点
  _pub_outlier_cloud = nh.advertise<sensor_msgs::PointCloud2>("/outlier_cloud", 1);

  // 垂直线个数16个
  nh.getParam("/lego_loam/laser/num_vertical_scans", _vertical_scans);
  // 水平点个点1800个
  nh.getParam("/lego_loam/laser/num_horizontal_scans", _horizontal_scans);
  // 下面角度最低的角度,-15度
  nh.getParam("/lego_loam/laser/vertical_angle_bottom", _ang_bottom);
  float vertical_angle_top;
  // 最上面一条线的角度,15度
  nh.getParam("/lego_loam/laser/vertical_angle_top", vertical_angle_top);


  nh.getParam("/lego_loam/camera/num_vertical_pixel", _num_vertical_pixel);
  nh.getParam("/lego_loam/camera/num_horizontal_pixel", _num_horizontal_pixel);
  nh.getParam("/lego_loam/camera/horizontal_fov", _horizontal_fov);
  nh.getParam("/lego_loam/camera/vertical_fov", _vertical_fov);



  // 水平像素点之间的间隔角度
  _ang_resolution_X = (_horizontal_fov) / (_num_horizontal_pixel - 1);
  // 垂直方向每个像素点的间隔rad值
  _ang_resolution_Y = DEG_TO_RAD*(_vertical_fov) / float(_num_vertical_pixel-1);

  // 计算最下面那条线的rad值
  _ang_bottom = -(_vertical_fov / 2) * DEG_TO_RAD;
  // x,y的分割因子
  _segment_alpha_X = _ang_resolution_X;
  _segment_alpha_Y = _ang_resolution_Y;

  // 得到分割阈值,60.0
  nh.getParam("/lego_loam/imageProjection/segment_theta", _segment_theta);
  // 分割值的rad值
  _segment_theta *= DEG_TO_RAD;

  // 
  nh.getParam("/lego_loam/imageProjection/segment_valid_point_num",
              _segment_valid_point_num);
  nh.getParam("/lego_loam/imageProjection/segment_valid_line_num",
              _segment_valid_line_num);

  // 地面扫面的index
  nh.getParam("/lego_loam/laser/ground_scan_index",
              _ground_scan_index);

  nh.getParam("/lego_loam/laser/sensor_mount_angle",
              _sensor_mount_angle);
  _sensor_mount_angle *= DEG_TO_RAD;

  // 所有点的总数
  const size_t cloud_size = _num_vertical_pixel * _num_horizontal_pixel;

  // 输入的点云
  _rgbd_cloud_in.reset(new pcl::PointCloud<PointType>());
  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  // 分割出的地面点云
  _ground_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
  _full_info_cloud->points.resize(cloud_size);

}

void ImageProjection::resetParameters() {
  const size_t cloud_size = _num_vertical_pixel * _num_horizontal_pixel;
  PointType nanPoint;
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();

  _laser_cloud_in->clear();
  _ground_cloud->clear();
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();

  _range_mat.resize(_num_vertical_pixel, _num_horizontal_pixel);
  _ground_mat.resize(_num_vertical_pixel, _num_horizontal_pixel);
  _label_mat.resize(_num_vertical_pixel, _num_horizontal_pixel);

  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
  std::fill(_full_info_cloud->points.begin(), _full_info_cloud->points.end(),
            nanPoint);

  _seg_msg.startRingIndex.assign(_num_vertical_pixel, 0);
  _seg_msg.endRingIndex.assign(_num_vertical_pixel, 0);

  _seg_msg.segmentedCloudGroundFlag.assign(cloud_size, false);
  _seg_msg.segmentedCloudColInd.assign(cloud_size, 0);
  _seg_msg.segmentedCloudRange.assign(cloud_size, 0);
}

void ImageProjection::cloudHandler(
    const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) {
  // Reset parameters
  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(*laserCloudMsg, *_laser_cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);
  _seg_msg.header = laserCloudMsg->header;

  findStartEndAngle();
  // Range image projection
  projectPointCloud();
  // Mark ground points
  groundRemoval();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();
}

// void ImageProjection::cloudHandler(
//     const pcl::PointCloud<PointTypeNew>::Ptr rgbd_cloud) {
//   // Reset parameters
//   resetParameters();

//   // Copy and remove NAN points
//   pcl::copyPointCloud(*rgbd_cloud, *_rgbd_cloud_in);
//   std::vector<int> indices;
//   pcl::removeNaNFromPointCloud(*_rgbd_cloud_in, *_rgbd_cloud_in, indices);
//   // _seg_msg.header = laserCloudMsg->header;

//   // findStartEndAngle();
//   // Range image projection
//   // projectPointCloud();
//   // Mark ground points
//   groundRemoval();
//   // Point cloud segmentation
//   cloudSegmentation();
//   //publish (optionally)
//   publishClouds();
// }

void ImageProjection::cloudHandler(
    const sensor_msgs::PointCloud2 &cloud) {
  // Reset parameters
  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(cloud, *_rgbd_cloud_in);
  // pcl::copyPointCloud(*cloud, *_rgbd_cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_rgbd_cloud_in, *_rgbd_cloud_in, indices);
  _seg_msg.header = cloud.header;

// sensor_msgs::PointCloud2 laserCloudTemp;
// pcl::toROSMsg(*_rgbd_cloud_in, laserCloudTemp);

//   laserCloudTemp.header.stamp = cloud.header.stamp;
//   laserCloudTemp.header.frame_id = "camera_init";
//   _pub_full_cloud.publish(laserCloudTemp);

  // findStartEndAngle();
  // Range image projection
  // projectPointCloud();
  // Mark ground points
  groundRemoval();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();
}

// 不用
void ImageProjection::projectPointCloud() {
  // range image projection
  const size_t cloudSize = _laser_cloud_in->points.size();

  for (size_t i = 0; i < cloudSize; ++i) {
    PointType thisPoint = _laser_cloud_in->points[i];

    float range = sqrt(thisPoint.x * thisPoint.x +
                       thisPoint.y * thisPoint.y +
                       thisPoint.z * thisPoint.z);

    // find the row and column index in the image for this point
    float verticalAngle = std::asin(thisPoint.z / range);
        //std::atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y));

    int rowIdn = (verticalAngle + _ang_bottom) / _ang_resolution_Y;
    if (rowIdn < 0 || rowIdn >= _vertical_scans) {
      continue;
    }

    float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);

    int columnIdn = -round((horizonAngle - M_PI_2) / _ang_resolution_X) + _horizontal_scans * 0.5;

    if (columnIdn >= _horizontal_scans){
      columnIdn -= _horizontal_scans;
    }

    if (columnIdn < 0 || columnIdn >= _horizontal_scans){
      continue;
    }

    if (range < 0.1){
      continue;
    }

    _range_mat(rowIdn, columnIdn) = range;

    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    size_t index = columnIdn + rowIdn * _horizontal_scans;
    _full_cloud->points[index] = thisPoint;
    // the corresponding range of a point is saved as "intensity"
    _full_info_cloud->points[index] = thisPoint;
    _full_info_cloud->points[index].intensity = range;
  }
}

// 不用
void ImageProjection::findStartEndAngle() {
  // start and end orientation of this cloud
  auto point = _laser_cloud_in->points.front();
  _seg_msg.startOrientation = -std::atan2(point.y, point.x);

  point = _laser_cloud_in->points.back();
  _seg_msg.endOrientation = -std::atan2(point.y, point.x) + 2 * M_PI;

  if (_seg_msg.endOrientation - _seg_msg.startOrientation > 3 * M_PI) {
    _seg_msg.endOrientation -= 2 * M_PI;
  } else if (_seg_msg.endOrientation - _seg_msg.startOrientation < M_PI) {
    _seg_msg.endOrientation += 2 * M_PI;
  }
  _seg_msg.orientationDiff =
      _seg_msg.endOrientation - _seg_msg.startOrientation;
}

// 修改完成
void ImageProjection::groundRemoval() {
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (size_t j = 0; j < _num_horizontal_pixel; ++j) {
    for (size_t i = _num_vertical_pixel / 2 - 1; i < _num_vertical_pixel - 1; ++i) {
      size_t lowerInd = j + (i)*_num_horizontal_pixel;
      size_t upperInd = j + (i + 1) * _num_horizontal_pixel;

      if (_rgbd_cloud_in->points[lowerInd].z == 0 ||
          _rgbd_cloud_in->points[upperInd].z == 0) {
        // no info to check, invalid points
        _ground_mat(i, j) = -1;
        continue;
      }

      float dX =
          _rgbd_cloud_in->points[upperInd].x - _rgbd_cloud_in->points[lowerInd].x;
      float dY =
          _rgbd_cloud_in->points[upperInd].y - _rgbd_cloud_in->points[lowerInd].y;
      float dZ =
          _rgbd_cloud_in->points[upperInd].z - _rgbd_cloud_in->points[lowerInd].z;

      float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY + dZ * dZ));

      // TODO: review this change

      if ( (vertical_angle - _sensor_mount_angle) >= 0.5 * DEG_TO_RAD) {
        _ground_mat(i, j) = 1;
        _ground_mat(i + 1, j) = 1;
      }
    }
  }
  // extract ground cloud (_ground_mat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~_N_scan-1, need _range_mat
  // for mark label matrix for the 16th scan
  for (size_t i = 0; i < _num_vertical_pixel; ++i) {
    for (size_t j = 0; j < _num_horizontal_pixel; ++j) {
      // 创建range_mat
      // generalRangeMat(i, j);
      if (_ground_mat(i, j) == 1 ||
          _rgbd_cloud_in->points[j + i * _num_horizontal_pixel].z == FLT_MAX || 
          _rgbd_cloud_in->points[j + i * _num_horizontal_pixel].z == 0 || 
          _ground_mat(i, j) == -1) {
          _label_mat(i, j) = -1;
      }
    }
  }

  for (size_t i = _num_vertical_pixel / 2 - 1; i < _num_vertical_pixel; ++i) {
    for (size_t j = 0; j < _num_horizontal_pixel; ++j) {
      if (_ground_mat(i, j) == 1) {
        _ground_cloud->points.push_back(_rgbd_cloud_in->points[j + i * _num_horizontal_pixel]);
      }
      // std::cout << "nihaoma!!!!!!!!!!!!" << std::endl;
        // _ground_cloud->push_back(_rgbd_cloud_in->points[j + i * _num_horizontal_pixel]);
        // _ground_cloud->points.push_back(_rgbd_cloud_in->points[j + i * _num_horizontal_pixel]);
    }
  }
  // std::cout << "zhixing!!!!!!!!!!!!!!!!!!!!------------------->" << _ground_cloud->points.size() << std::endl;
}
// 创建_range_mat的矩阵
void ImageProjection::generalRangeMat(const int &row, const int &col) {
  PointType thisPoint = _rgbd_cloud_in->points[col + row * _num_horizontal_pixel];
  float range = sqrt(thisPoint.x * thisPoint.x +
                       thisPoint.y * thisPoint.y +
                       thisPoint.z * thisPoint.z);
  _range_mat(row, col) = range;
}

// 
void ImageProjection::cloudSegmentation() {
  // segmentation process
  // 如果lab的mat不是地面点和无效点,就放入labelComponents函数进行分割
  for (size_t i = 0; i < _num_vertical_pixel; ++i)
    for (size_t j = 0; j < _num_horizontal_pixel; ++j)
      if (_label_mat(i, j) == 0) labelComponents(i, j);

  // 
  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (size_t i = 0; i < _num_vertical_pixel; ++i) {
    // _seg_msg.startRingIndex[i] = sizeOfSegCloud - 1 + 5;

    for (size_t j = 0; j < _num_horizontal_pixel; ++j) {
      if (_label_mat(i, j) > 0 || _ground_mat(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (_label_mat(i, j) == 999999) {
          if (i < (_num_vertical_pixel / 2 - 1) && j % 5 == 0) {
            _outlier_cloud->push_back(
                _rgbd_cloud_in->points[j + i * _num_horizontal_pixel]);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        if (_ground_mat(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < _num_horizontal_pixel - 5) continue;
        }
        // // mark ground points so they will not be considered as edge features
        // // later
        // _seg_msg.segmentedCloudGroundFlag[sizeOfSegCloud] =
        //     (_ground_mat(i, j) == 1);
        // // mark the points' column index for marking occlusion later
        // _seg_msg.segmentedCloudColInd[sizeOfSegCloud] = j;
        // // save range info
        // _seg_msg.segmentedCloudRange[sizeOfSegCloud] =
        //     _range_mat(i, j);
        // save seg cloud
        _segmented_cloud->push_back(_rgbd_cloud_in->points[j + i * _num_horizontal_pixel]);
        // size of seg cloud
        // ++sizeOfSegCloud;
      }
    }

    // _seg_msg.endRingIndex[i] = sizeOfSegCloud - 1 - 5;
  }

  // extract segmented cloud for visualization
  for (size_t i = 0; i < _num_vertical_pixel; ++i) {
    for (size_t j = 0; j < _num_horizontal_pixel; ++j) {
      if (_label_mat(i, j) > 0 && _label_mat(i, j) != 999999) {
        _segmented_cloud_pure->push_back(
            _rgbd_cloud_in->points[j + i * _num_horizontal_pixel]);
        _segmented_cloud_pure->points.back().intensity =
            _label_mat(i, j);
      }
    }
  }
}

void ImageProjection::labelComponents(int row, int col) {
  // 分割阈值是60度*rad
  const float segmentThetaThreshold = tan(_segment_theta);

  std::vector<bool> lineCountFlag(_num_vertical_pixel, false);
  const size_t cloud_size = _num_vertical_pixel * _num_horizontal_pixel;
  using Coord2D = Eigen::Vector2i;
  boost::circular_buffer<Coord2D> queue(cloud_size);
  boost::circular_buffer<Coord2D> all_pushed(cloud_size);

  queue.push_back({ row,col } );
  all_pushed.push_back({ row,col } );

  const Coord2D neighborIterator[4] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (queue.size() > 0) {
    // Pop point
    Coord2D fromInd = queue.front();
    queue.pop_front();

    // Mark popped point
    _label_mat(fromInd.x(), fromInd.y()) = _label_count;
    // Loop through all the neighboring grids of popped grid

    for (const auto& iter : neighborIterator) {
      // new index
      int thisIndX = fromInd.x() + iter.x();
      int thisIndY = fromInd.y() + iter.y();
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= _num_vertical_pixel){
        continue;
      }
      // at range image margin (left or right side)
      if (thisIndY < 0){
        thisIndY = _num_horizontal_pixel - 1;
      }
      if (thisIndY >= _num_horizontal_pixel){
        thisIndY = 0;
      }
      // prevent infinite loop (caused by put already examined point back)
      if (_label_mat(thisIndX, thisIndY) != 0){
        continue;
      }

      float d1 = std::max(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));
      float d2 = std::min(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));

      float alpha = (iter.x() == 0) ? _ang_resolution_X : _ang_resolution_Y;
      float tang = (d2 * sin(alpha) / (d1 - d2 * cos(alpha)));
      // ??????阈值设置多少合适
      if (tang > segmentThetaThreshold) {
        queue.push_back( {thisIndX, thisIndY } );

        _label_mat(thisIndX, thisIndY) = _label_count;
        lineCountFlag[thisIndX] = true;

        all_pushed.push_back(  {thisIndX, thisIndY } );
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  if (all_pushed.size() >= 30){
    feasibleSegment = true;
  }
  else if (all_pushed.size() >= _segment_valid_point_num) {
    int lineCount = 0;
    for (size_t i = 0; i < _vertical_scans; ++i) {
      if (lineCountFlag[i] == true) ++lineCount;
    }
    if (lineCount >= _segment_valid_line_num) feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true) {
    ++_label_count;
  } else {  // segment is invalid, mark these points
    for (size_t i = 0; i < all_pushed.size(); ++i) {
      _label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
    }
  }
}

void ImageProjection::publishClouds() {
  const auto& cloudHeader = _seg_msg.header;

  sensor_msgs::PointCloud2 laserCloudTemp;

  auto PublishCloud = [&](ros::Publisher& pub,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (pub.getNumSubscribers() != 0) {
      pcl::toROSMsg(*cloud, laserCloudTemp);
      laserCloudTemp.header.stamp = cloudHeader.stamp;
      // laserCloudTemp.header.frame_id = "base_link";
      laserCloudTemp.header.frame_id = "camera_init";
      pub.publish(laserCloudTemp);
    }
  };

  // auto PublishCloudRGB = [&](ros::Publisher& pub,
  //                         const pcl::PointCloud<PointTypeNew>::Ptr& cloud) {
  //   if (pub.getNumSubscribers() != 0) {
  //     pcl::toROSMsg(*cloud, laserCloudTemp);
  //     laserCloudTemp.header.stamp = cloudHeader.stamp;
  //     laserCloudTemp.header.frame_id = "base_link";
  //     pub.publish(laserCloudTemp);
  //   }
  // };

  PublishCloud(_pub_outlier_cloud, _outlier_cloud);
  PublishCloud(_pub_segmented_cloud, _segmented_cloud);
  // PublishCloud(_pub_full_cloud, _rgbd_cloud_in);
  PublishCloud(_pub_ground_cloud, _ground_cloud);
  PublishCloud(_pub_segmented_cloud_pure, _segmented_cloud_pure);
  PublishCloud(_pub_full_info_cloud, _full_info_cloud);

  // if (_pub_segmented_cloud_info.getNumSubscribers() != 0) {
  //   _pub_segmented_cloud_info.publish(_seg_msg);
  // }

  // //--------------------
  // ProjectionOut out;
  // out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  // out.segmented_cloud.reset(new pcl::PointCloud<PointType>());

  // std::swap( out.seg_msg, _seg_msg);
  // std::swap(out.outlier_cloud, _outlier_cloud);
  // std::swap(out.segmented_cloud, _segmented_cloud);

  // _output_channel.send( std::move(out) );

}


