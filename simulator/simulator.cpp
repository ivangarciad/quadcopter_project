/* \author Ivan Garcia Daza */

#include <iostream>
#include <unistd.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include <iostream>
#include "rs232.h"

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int corner, float angle_x, float angle_y, float angle_z);
void printUsage (const char* progName);

/** 
  Brieft coment about the function. 
  @param x x coment
  @param y y coment
  @param st st coment

  @return Void value is returned. 
**/ 
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }

  bool shapes(false); 

  if (pcl::console::find_argument (argc, argv, "-a") >= 0)
  {
    shapes = true;
    std::cout << "Shapes visualisation example\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  int port_num = 16;
  int bps = 19200;

  RS232_OpenComport(port_num, bps);

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Genarating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 20.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back (point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }

  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  int corner = 1.0;
  float angle_x = 0.0;
  float angle_y = 0.0;
  float angle_z = 0.0;
  float angle_x_1 = 0.0;
  float angle_y_1 = 0.0;
  float angle_z_1 = 0.0;

  if (shapes)
  {
    viewer = shapesVis(point_cloud_ptr, corner, angle_x, angle_y, angle_z);
  }

  unsigned char buffer[50];
  const float x = 1;
  
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    int aux = RS232_PollComport(port_num, (unsigned char*) &(buffer), sizeof(buffer));
    //cout << "Readed Bytes: " << buffer << '\n';

    unsigned char* start_ptr = (unsigned char*)memchr(buffer, '\n', sizeof(buffer));

    if (start_ptr != NULL )
    {
      unsigned char* end_ptr = (unsigned char*)memchr(start_ptr+1, '\n', sizeof(buffer));
      if (end_ptr != NULL && end_ptr - start_ptr == 26)
      {
        int ret = sscanf((const char*)(start_ptr+1), "%f;%f;%f;", &angle_x, &angle_y, &angle_z);
        
        if (angle_x > -360.0 && angle_x < 360.0 &&
            angle_y > -360.0 && angle_y < 360.0 &&
            angle_z > -360.0 && angle_z < 360.0)
        {
          //cout << "Line: " << start_ptr+1 << '\n';
          cout << "Angles: " << angle_x << ';' << angle_y << ';' << angle_z << '\n';
          
          angle_x = (1-x)*angle_x_1 + x*angle_x;
          angle_y = (1-x)*angle_y_1 + x*angle_y;
          angle_z = (1-x)*angle_z_1 + x*angle_z;
          
          angle_x_1 = angle_x;
          angle_y_1 = angle_y;
          angle_z_1 = angle_z;
        }
      }
    }

    viewer->spinOnce (20);

    boost::this_thread::sleep (boost::posix_time::microseconds (10000));
    viewer->removeShape("cube");
    viewer = shapesVis(point_cloud_ptr, corner, 2*angle_x, 2*angle_y, angle_z);

  }

  RS232_CloseComport(port_num);
}

/** 
  Brieft coment about the function. 
  @param x x coment
  @param y y coment
  @param st st coment

  @return Void value is returned. 
**/ 
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, int corner, float angle_x, float angle_y, float angle_z)
{

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  //viewer->addLine<pcl::PointXYZRGB> (cloud->points[0], cloud->points[cloud->size() - 1], "line");

  //viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  Eigen::Vector3f translation(0.0, 0.0, 0.0);
  //Eigen::Quaternionf rotation(0.9238795325112867, 0.0, -0.3826834323650897, 0.0);
  Eigen::Quaternionf rotation(0.0, 0.0, 0.0, 0.0);
  rotation = Eigen::AngleAxisf(angle_x*3.14/180, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(angle_y*3.14/180, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(angle_z*3.14/180, Eigen::Vector3f::UnitZ());

  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (corner);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.5);
  //viewer->addCube (coeffs, "cube");

  viewer->addCube (translation, rotation, 1.0, 1.0, 0.1, "cube");

  viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cube");
  coeffs.values.clear ();

  return (viewer);
}

/** 
  Brieft coment about the function. 
  @param x x coment
  @param y y coment
  @param st st coment

  @return Void value is returned. 
**/ 
void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-a           Shapes visualisation example\n"
            << "\n\n";
}

