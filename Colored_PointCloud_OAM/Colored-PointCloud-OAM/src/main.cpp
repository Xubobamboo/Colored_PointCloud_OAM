#include "featureAssociation.h"
#include "imageProjection.h"
#include "mapOptimization.h"
#include "transformFusion.h"
#include <chrono>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

// #include <opencv/cv.hpp>
// xubo
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.hpp>

#include<sys/time.h>

#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/visualization/cloud_viewer.h>

// develop branch
// new branch

void showPCL_single(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCA)
{
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("name"));
    pcl::visualization::CloudViewer viewer("pcd viewer");
	  viewer.showCloud(PCA);
    // system("pause");
    while (!viewer.wasStopped ())
    {
    }

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(PCA, 0, 255, 0); // green
    // viewer->setBackgroundColor(255, 255, 255);
    // //创建窗口
    // int vp;
    // viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
    // viewer->addCoordinateSystem(1.0);


    // viewer->addPointCloud<pcl::PointXYZRGB>(PCA, single_color, "source");

    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    // viewer->spin();
}

void returnPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PC, std::string &rgbPath, std::string &depthPath) {
    cv::Mat rgbMat, depthMat;
    pcl::PointXYZRGB Point;

    rgbMat = cv::imread(rgbPath, -1);
    depthMat = cv::imread(depthPath, -1);
    // cv::imshow("rgb", rgbMat);
    // cv::waitKey();

    if (rgbMat.size() != depthMat.size())
    {
        cout << "Color and depth image do not have the same resolution." << endl;
    } else
    {
        cout << "is ok!" << endl;
    }
    

    for (int i = 0; i < rgbMat.rows; i++)
    {
        for (int j = 0; j < rgbMat.cols; j++)
        {
            int b = rgbMat.at<cv::Vec3b>(i, j)[0];
            int g = rgbMat.at<cv::Vec3b>(i, j)[1]; 
            int r = rgbMat.at<cv::Vec3b>(i, j)[2];
            ushort d = depthMat.at<ushort>(i, j);
            // std::cout << depthMat.at<ushort>(i, j) << std::endl;
            Point.z = double(d) / SCALING_FACTOR;
            Point.x = (j - CENTER_X) * Point.z / FOCAL_LENGTH;
            Point.y = (i - CENTER_Y) * Point.z / FOCAL_LENGTH;
            Point.r = r;
            Point.g = g;
            Point.b = b;
            PC->push_back(Point);
        }
    }

    // for (int i = 0; i < depthMat.rows; i++)
    // {
    //     for (int j = 0; j < depthMat.cols; j++)
    //     {
    //         double d = depthMat.at<uchar>(i, j);
    //         cout << d << endl;
    //     }
    // }
    

    // rgbMat = cv::imread(__depthPath);
    // cv::imshow("depth", depthMat);
}

void returnPointCloud1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr PC, cv::Mat rgbMat, std::vector<ushort> depthMat) {
    pcl::PointXYZRGB Point;
    // cv::imshow("rgb", rgbMat);
    // cv::waitKey();

    // if (rgbMat.size() != depthMat.size())
    // {
    //     cout << "Color and depth image do not have the same resolution." << endl;
    // } else
    // {
    //     cout << "is ok!" << endl;
    // }
    

    for (int i = 0; i < rgbMat.rows; i++)
    {
        for (int j = 0; j < rgbMat.cols; j++)
        {
            int b = rgbMat.at<cv::Vec3b>(i, j)[0];
            int g = rgbMat.at<cv::Vec3b>(i, j)[1]; 
            int r = rgbMat.at<cv::Vec3b>(i, j)[2];
            // ushort d = depthMat.at<ushort>(i, j);
            // float d = depthMat.ptr<float>(i)[j];
            ushort d = depthMat[i * rgbMat.cols + j];
            Point.z = double(d) / SCALING_FACTOR;
            if (Point.z == 0) continue;
            Point.x = (j - CENTER_X) * Point.z / FOCAL_LENGTH;
            Point.y = (i - CENTER_Y) * Point.z / FOCAL_LENGTH;
            Point.r = r;
            Point.g = g;
            Point.b = b;
            PC->push_back(Point);
        }
    }

    // for (int i = 0; i < depthMat.rows; i++)
    // {
    //     for (int j = 0; j < depthMat.cols; j++)
    //     {
    //         double d = depthMat.at<uchar>(i, j);
    //         cout << d << endl;
    //     }
    // }
    

    // rgbMat = cv::imread(__depthPath);
    // cv::imshow("depth", depthMat);
}

ushort Little(char c0, char c1) {
  ushort num;
  num = c0 << 8 | c1;
  return num;
}

unsigned short BLEndianUshort(unsigned short value)
{
    return ((value & 0x00FF) << 8 ) | ((value & 0xFF00) >> 8);
}

// typedef union USHORT_UNION {
//   ushort num;
//   char c[2];
// } ushort_union;

// ushort Little(char c0, char c1) {
//   ushort_union tmp;
//   tmp.c[0] = c0;
//   tmp.c[1] = c1;
//   return tmp.num;
// }

// cv::Mat decodeDepthImage(const sensor_msgs::Image::ConstPtr& depth) {
//   // cv::Mat _depth;
//   // vector<ushort> depthList;
//   cv::Mat dep_im = cv::Mat::zeros(depth->height, depth->width, CV_16UC1);
//   for (int i = 0; i < depth->height; i++) {
//     for (int j = 0; j < depth->width; j++) {
//       int index = i * depth->width + j;
//       ushort tmp = Little(depth->data[index * 2], depth->data[index * 2 + 1]);
//       dep_im.at<ushort>(i, j) = tmp;
//     }
//   }
//   return dep_im;
// }

std::vector<ushort> decodeDepthImage(const sensor_msgs::Image::ConstPtr& depth) {
  // cv::Mat _depth;
  std::vector<ushort> depthList;
  for (int i = 0; i < depth->height; i++) {
    for (int j = 0; j < depth->width; j++) {
      int index = i * depth->width + j;
      ushort tmp = Little(depth->data[index * 2 + 2], depth->data[index * 2 + 3]);
      ushort tmp1 = BLEndianUshort(tmp);
      depthList.push_back(tmp1);
    }
  }
  return depthList;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rgbd_oam");

  ros::NodeHandle nh("~");
  std::string aligned_associate;
  std::string imu_topic;
  std::string lidar_topic;

  // xubo
  std::string image_topic;
  std::string depth_topic;

  nh.getParam("rosbag", rosbag);
  nh.getParam("imu_topic", imu_topic);
  nh.getParam("lidar_topic", lidar_topic);

  // xubo
  nh.getParam("image_topic", image_topic);
  nh.getParam("depth_topic", depth_topic);


  bool use_rosbag = false;

  rosbag::Bag bag;

  if (!rosbag.empty()) {
    try {
      bag.open(rosbag, rosbag::bagmode::Read);
      use_rosbag = true;
    } catch (std::exception& ex) {
      ROS_FATAL("Unable to open rosbag [%s]", rosbag.c_str());
      return 1;
    }
  }

  // 创建投影输出
  Channel<ProjectionOut> projection_out_channel(true);
  // 线特征和面特征计算后输出
  Channel<AssociationOut> association_out_channel(use_rosbag);

  // // 初始化投影点类
  // ImageProjection IP(nh, projection_out_channel);

  // FeatureAssociation FA(nh, projection_out_channel,
  //                       association_out_channel);

  // MapOptimization MO(nh, association_out_channel);

  // TransformFusion TF(nh);



  ROS_INFO("\033[1;32m---->\033[0m LeGO-LOAM Started.");

  if( !use_rosbag ){
    ROS_INFO("SPINNER");
    ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
    spinner.spin();
  } else {
    ROS_INFO("ROSBAG");
    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(lidar_topic);

    // xubo
    topics.push_back(image_topic);
    topics.push_back(depth_topic);

    std::cout << "do this !!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    auto start_real_time = std::chrono::high_resolution_clock::now();
    auto start_sim_time = view.getBeginTime();

    auto prev_real_time = start_real_time;
    auto prev_sim_time = start_sim_time;

    auto clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock",1);

    // xubo
    cv_bridge::CvImagePtr cv_rgb8 = nullptr;
    cv_bridge::CvImagePtr cv_32fc1 = nullptr;
    cv::Mat rgb;
    // cv::Mat depth;
    std::vector<ushort> depthList;


    ros::Publisher _pub_full_cloud_origen;
    // 发布所有点云
    _pub_full_cloud_origen = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected_origen", 2);
    
    ros::Time stamp_rgb8;
    ros::Time stamp_32fc1;
    for(const rosbag::MessageInstance& m: view)
    {
      // 读取image类型
      const sensor_msgs::ImageConstPtr &image = m.instantiate<sensor_msgs::Image>(); 
      if (image != NULL){
        if (image->encoding == "rgb8") {
          if (cv_rgb8 == nullptr) {
            stamp_rgb8 = image->header.stamp;
            cv_rgb8 = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
            rgb = cv_rgb8->image;
          }
        }
        if (image->encoding == "32FC1") {
          if (cv_32fc1 == nullptr) {
            stamp_32fc1 = image->header.stamp;
            cv_32fc1 = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
            depthList = decodeDepthImage(image);
            // cv::Mat img = cv_32fc1->image;
          }
        }
        if (cv_rgb8 != nullptr && cv_32fc1 != nullptr) {
          double time_rgb8 = stamp_rgb8.toSec();
          double time_32fc1 = stamp_32fc1.toSec();
          double delta_stamp = time_32fc1 - time_rgb8;
          if (fabs(delta_stamp) < 0.01) {
            // rgbd_cloud->points.clear();
            pcl::PointCloud<PointTypeNew>::Ptr rgbd_cloud(new pcl::PointCloud<PointTypeNew>);
            // pcl::PointCloud<PointTypeNew>::Ptr cloud(new pcl::PointCloud<PointTypeNew>);
            // ImageToPointCloud(cv_rgb8->image, cv_32fc1->image, rgbd_cloud);
            // // ImageToPointCloud(cv_32fc1->image, cloud);
            // sensor_msgs::PointCloud2 cloudTemp;
            // pcl::toROSMsg(*rgbd_cloud, cloudTemp);
            // cloudTemp.header.stamp = stamp_32fc1;
            // // cloudTemp.header.frame_id = "base_link";
            // // cloudTemp.header.frame_id = "camera_init";
            // cloudTemp.header.frame_id = "map";
            // _pub_full_cloud_origen.publish(cloudTemp);

            // for( size_t nrow = 0; nrow < depth.rows; nrow++)  
            // {  
            //     for(size_t ncol = 0; ncol < depth.cols; ncol++)  
            //     {  
            //       std::cout << depth.ptr<ushort>(nrow)[ncol] << std::endl;
            //       // float d = depthMat.ptr<ElementType>(nrow)[ncol];      
            //     }  
            // } 

            std::cout << "time_rgb8----------------------------->" << time_rgb8 << std::endl;
            std::cout << "time_32fc1----------------------------->" << time_32fc1 << std::endl;
            std::cout << "stamp_rgb8----------------------------->" << stamp_rgb8 << std::endl;
            std::cout << "stamp_32fc1----------------------------->" << stamp_32fc1 << std::endl;

            std::string string1 = "/home/gabriel/Datas/data/rgbd_dataset_freiburg1_plant/rgb/1305032354.025237.png";
            std::string string2 = "/home/gabriel/Datas/data/rgbd_dataset_freiburg1_plant/depth/1305032354.009456.png";

            // returnPointCloud(rgbd_cloud, string1, string2);
            returnPointCloud1(rgbd_cloud, rgb, depthList);

            showPCL_single(rgbd_cloud);

            // IP.cloudHandler(cloudTemp);
            // std::cout << "delta_stamp ---------------------------->" << delta_stamp << std::endl;
            cv_rgb8 = nullptr;
            cv_32fc1 = nullptr;
          } else if (delta_stamp > 0 && fabs(delta_stamp) < 1) {
            cv_rgb8 = nullptr;
          } else if (delta_stamp < 0 && fabs(delta_stamp) < 1) {
            cv_32fc1 = nullptr;
          } else {
            cv_rgb8 = nullptr;
            cv_32fc1 = nullptr;
          }
        }
      }

      // const sensor_msgs::PointCloud2ConstPtr &cloud = m.instantiate<sensor_msgs::PointCloud2>(); 
      // if (cloud != NULL){
      //   IP.cloudHandler(cloud);
      //   // ROS_INFO("cloud");
      // }

      // const sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
      // if (imu != NULL){
      //   number1++;
      //   std::cout << "do this ----------------imu------------>" << number1 << std::endl;
      //   // 没有imu数据的时候不调用
      //   FA.imuHandler(imu);
      //   MO.imuHandler(imu);
      //  // ROS_INFO("imu");
      // }

      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock = m.getTime();
      clock_publisher.publish( clock_msg );

      auto real_time = std::chrono::high_resolution_clock::now();
      if( real_time - prev_real_time > std::chrono::seconds(5) )
      {
        auto sim_time = m.getTime();
        auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time-prev_real_time).count()*0.001;
        auto delta_sim = (sim_time - prev_sim_time).toSec();
        ROS_INFO("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);
        prev_sim_time = sim_time;
        prev_real_time = real_time;
      }
      ros::spinOnce();
    }

    bag.close();

    auto real_time = std::chrono::high_resolution_clock::now();
    auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time-start_real_time).count()*0.001;
    auto delta_sim = (view.getEndTime() - start_sim_time).toSec();
    ROS_INFO("Entire rosbag processed at %.1fX speed", delta_sim / delta_real);
  }


  // must be called to cleanup threads
  ros::shutdown();

  return 0;
}


