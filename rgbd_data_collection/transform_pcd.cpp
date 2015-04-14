#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cstdio>
#include <boost/filesystem.hpp>


int
main (int argc, char** argv)
{
  std::vector<float> t_xyz;
  std::vector<float> q_wxyz;
  std::vector<std::string> pcd_filenames;
  int got_t_xyz = pcl::console::parse_x_arguments(argc, argv, "--xyz", t_xyz);
  int got_q_wxyz = pcl::console::parse_x_arguments(argc, argv, "--wxyz", q_wxyz);
  int replace = pcl::console::find_argument(argc, argv, "--replace");
  int generate_ply = pcl::console::find_argument(argc, argv, "--generate-ply");
  int pcd_arg = pcl::console::find_argument(argc, argv, "--pcds");
  for(int a=pcd_arg+1; a<argc; a++) {
    pcd_filenames.push_back(std::string(argv[a]));
  }

  if(got_t_xyz == -1 or got_q_wxyz == -1 or pcd_filenames.size() == 0 or t_xyz.size() != 3 or q_wxyz.size() != 4) {
    PCL_ERROR("Bad args!\n");
    std::cerr<<"xyz: ";
    for(int i=0; i<t_xyz.size(); i++) std::cerr<<t_xyz[i]<<" ";
    std::cerr<<std::endl;
    std::cerr<<"wxyz: ";
    for(int i=0; i<q_wxyz.size(); i++) std::cerr<<q_wxyz[i]<<" ";
    std::cerr<<std::endl;
    std::cerr<<"pcds: ";
    for(int i=0; i<pcd_filenames.size(); i++) std::cerr<<pcd_filenames[i]<<" ";
    std::cerr<<std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  int pcd_index = 0;
  for(std::vector<std::string>::iterator it=pcd_filenames.begin();
      it != pcd_filenames.end();
      ++it)
  {
    std::string pcd_filename = *it;

    boost::filesystem::path pcd_path(pcd_filename);

    std::cerr<<"["<<++pcd_index<<"/"<<pcd_filenames.size()<<"] Processing: "<<pcd_filename<<std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_filename, *cloud) == -1) //* load the file
    {
      PCL_ERROR(("Couldn't read file: "+pcd_filename+"\n").c_str());
      rename(pcd_filename.c_str(), std::string("broken_"+pcd_filename).c_str());
      continue;
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(Eigen::Vector3f(t_xyz[0],t_xyz[1],t_xyz[2]));
    transform.rotate(Eigen::Quaternion<float>(q_wxyz[0], q_wxyz[1], q_wxyz[2], q_wxyz[3]));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

    boost::filesystem::path path = pcd_path.parent_path();
    std::string stem = pcd_path.stem().string();
    std::string filename = pcd_path.filename().string();

    if(replace == -1)  {
      pcl::io::savePCDFileASCII((path/("transformed_"+filename)).string(), *transformed_cloud);

      if(generate_ply != -1) {
        pcl::io::savePLYFileASCII((path/(stem+".ply")).string(), *cloud);
        pcl::io::savePLYFileASCII((path/("transformed_"+stem+".ply")).string(), *transformed_cloud);
      }
    } else {
      pcl::io::savePCDFileASCII((path/("orig_"+filename)).string(), *cloud);
      pcl::io::savePCDFileASCII(pcd_filename, *transformed_cloud);
      if(generate_ply != -1) {
        pcl::io::savePLYFileASCII((path/("orig_"+stem+".ply")).string(), *cloud);
        pcl::io::savePLYFileASCII((path/(stem+".ply")).string(), *transformed_cloud);
      }
    }
  }

  return (0);
}

