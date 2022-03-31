#include <vector>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/visualization/cloud_viewer.h>
#include <Eigen/Eigenvalues> 
using namespace  std;
vector<Eigen::Vector2d> m_points;  // 点集S
// vector<pair<Eigen::Vector2d, Eigen::Vector2d>> m_edges;   // 边
vector<Eigen::Vector2d> shape_points;
double m_radius = 0.1;

void AlphaShapes()
{
    // m_edges.clear();
    shape_points.clear();
    for(int i=0; i<m_points.size(); i++)
    {
        // k从i+1开始，减少重复计算
        for(int k=i+1; k<m_points.size(); k++)
        {
            // 跳过距离大于直径的情况
            if ((m_points.at(i) - m_points.at(k)).norm() > m_radius)
            {
                continue;
            }
            // 两个圆心
            Eigen::Vector2d c1,c2;
 
            // 线段中点
            Eigen::Vector2d center = 0.5*(m_points[i]+m_points[k]);
 
            // 方向向量 d = (x,y)
            Eigen::Vector2d dir = m_points[i] - m_points[k];
 
            // 垂直向量 n = (a,b)  a*dir.x+b*dir.y = 0; a = -(b*dir.y/dir.x)
            Eigen::Vector2d normal;
            normal(1) = 5;
 
            if(dir.x() != 0)
            {
                // normal.setX(-(normal.y()*dir.y())/dir.x());
                normal(0) = - normal(1)*dir(1)/dir(0);
            }    
            else     // 如果方向平行于y轴
            {
                normal(0) = 1;
                normal(1) = 0;
                // normal.setX(1);
                // normal.setY(0);
            }
            normal.normalize();   // 法向量单位化
 
            float len = sqrt(m_radius*m_radius-(0.25*dir.norm()*dir.norm()));
            c1 = center + len*normal;
            c2 = center - len*normal;
 
            // b1、b2记录是否在圆C1、C2中找到其他点。
            bool b1 = false, b2 = false;
            for (size_t m = 0; m < m_points.size(); m++)
            {
              if( m == i || m == k)
                    continue;
              if((m_points.at(m) - c1).norm() < m_radius)
              {
                b1 = true;
                break;
              } 
            }

            for (size_t m = 0; m < m_points.size(); m++)
            {
              if( m == i || m == k)
                    continue;
              if((m_points.at(m) - c2).norm() < m_radius)
              {
                b2 = true;
                break;
              }
            }
            if (!b1 || !b2)
            {
              shape_points.emplace_back(m_points.at(k));
            }
            // for(int m=0; m<m_points.size(); m++)
            // {
            //     if( m == i || m == k)
            //         continue;
            //     if(b1!=true && (m_points.at(m) - c1).norm() < m_radius)
            //         b1 = true;
            //     if(b2!=true && (m_points.at(m) - c2).norm() < m_radius)
            //         b2 = true;
            //     // 如果都有内部点，不必再继续检查了
            //     if(b1 == true && b2 == true)
            //         break;
            // }
            // pair<Eigen::Vector2d, Eigen::Vector2d>  edge;
            // if(b1!=true || b2!=true)
            // {
            //     edge.first = m_points.at(i);
            //     edge.second = m_points.at(k);
            //     m_edges.push_back(edge);
            // }
 
        }
    }
 
}

void getPlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr planepoints, Eigen::Vector3d& normal, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ& center)
{
  planepoints->reserve(cloud->size());
  // std::cout<<"1"<<std::endl;
  Eigen::Matrix3d M = Eigen::Matrix3d::Zero(3,3);
  // std::cout<<"1"<<std::endl;
  // pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
  pcl::PointCloud<pcl::PointXYZ>::iterator iter_point;
  // std::cout<<"1"<<std::endl;
  for (iter_point = cloud->begin(); iter_point != cloud->end(); iter_point++)
  {   
    // std::cout<<"2"<<std::endl;
    Eigen::Vector3d ve((*iter_point).x - center.x, (*iter_point).y - center.y, (*iter_point).z - center.z);
    M += ve*ve.transpose();
  }
  // std::cout<<"get M matrix"<<std::endl;
  Eigen::EigenSolver<Eigen::Matrix3d> es(M);
  Eigen::Matrix3d::Index b;
  auto minEigenValue = es.eigenvalues().real().minCoeff(&b);
  double eigenValuesSum = es.eigenvalues().real().sum();
  normal = es.eigenvectors().real().col(b);
  // std::cout<<"get normal"<<std::endl;
  Eigen::Vector3d center_(center.x, center.y, center.z);
  double d = -(normal.dot(center_));
  
  for (iter_point = cloud->begin(); iter_point != cloud->end(); iter_point++)
  {
    Eigen::Vector3d point((*iter_point).x, (*iter_point).y, (*iter_point).z);
    double dis = normal.dot(point) + d;
    Eigen::Vector3d pointShape = point - dis*normal;
    pcl::PointXYZ p(pointShape(0), pointShape(1), pointShape(2));
    planepoints->emplace_back(p);
  }
  // std::cout<<"get plane points"<<std::endl;
}

vector<Eigen::Vector3d> fitPoly(pcl::PointCloud<pcl::PointXYZ>& cloud_hull, Eigen::Vector3d & normal, Eigen::Vector3d & center_eigen)
{
  Eigen::Vector3d z_axid = normal;
  Eigen::Vector3d x_point;
  for (auto & iter:(cloud_hull))
  {
      Eigen::Vector3d tmppoint(iter.x, iter.y, iter.z);
      if ((tmppoint - center_eigen).norm() > 0.2)
      {
          x_point = tmppoint;
          break;
      }
  }
  cout<<"the cor point is "<<x_point(0)<<" "<<x_point(1)<<" "<<x_point(2)<<endl;
  Eigen::Vector3d x_axid = (x_point - center_eigen).normalized();
  Eigen::Vector3d y_axid = (normal.cross(x_axid)).normalized();

  cout<<"x : "<<x_axid.transpose()<<endl;
  cout<<"y : "<<y_axid.transpose()<<endl;
  cout<<"z : "<<z_axid.transpose()<<endl;
  // 从定义的平面坐标系到世界坐标系
  Eigen::Matrix3d rotation2W;

  rotation2W<<x_axid.dot(Eigen::Vector3d::UnitX()), y_axid.dot(Eigen::Vector3d::UnitX()), 
              z_axid.dot(Eigen::Vector3d::UnitX()), x_axid.dot(Eigen::Vector3d::UnitY()),
              y_axid.dot(Eigen::Vector3d::UnitY()), z_axid.dot(Eigen::Vector3d::UnitY()),
              x_axid.dot(Eigen::Vector3d::UnitZ()), y_axid.dot(Eigen::Vector3d::UnitZ()),
              z_axid.dot(Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d T1=Eigen::Isometry3d::Identity();
  T1.rotate (rotation2W);
  T1.pretranslate (center_eigen);
//   std::list<Point> points;
//   for (auto & iter:(cloud_hull))
//   {
//     Eigen::Vector3d new_p = T1.inverse()*Eigen::Vector3d(iter.x, iter.y, iter.z);
//     points.emplace_back(Point(new_p(0), new_p(1)));
//   }
//   Alpha_shape_2 A(points.begin(), points.end(), FT(0.1), Alpha_shape_2::GENERAL);
//   std::vector<Segment> segments;
//   alpha_edges(A, std::back_inserter(segments));

  for (auto & iter:(cloud_hull))
  {
    Eigen::Vector3d new_p = T1.inverse()*Eigen::Vector3d(iter.x, iter.y, iter.z);
    m_points.emplace_back(new_p.head(2));
  }
  cout<<"points size is "<<m_points.size()<<endl;
  AlphaShapes();
  cout<<"shape points size is "<<shape_points.size()<<endl;
  // for (auto & iter_segment : m_edges)
  // {
  //   cout<<"start : "<<iter_segment.first.transpose()<<endl;
  //   cout<<"endl : "<<iter_segment.second.transpose()<<endl;
  // }
  vector<Eigen::Vector3d> edgePoints;
  // for (auto & iter_segment : m_edges)
  // {
  //   Eigen::Vector3d tmpp(iter_segment.first(0), iter_segment.second(1), 0.0);
  //   cout<<"start : "<<iter_segment.first.transpose()<<endl;
  //   edgePoints.emplace_back(T1 * tmpp);
  // }
  for (auto & shape_point : shape_points)
  {
    Eigen::Vector3d tmpp(shape_point(0), shape_point(1), 0.0);
    edgePoints.emplace_back(T1 * tmpp);
  }
  
  for (auto & point : edgePoints)
  {
    cout<<point.transpose()<<endl;
  }
  return edgePoints;
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //*打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../5980_0.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file rabbit.pcd\n");
        return(-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);//给滤波对象设置需过滤的点云
    sor.setLeafSize(0.1f, 0.1f, 0.1f);//设置滤波时创建的体素大小为1cm*1cm*1cm的立方体
    sor.filter(*cloud_filtered);//执行滤波处理，存储输出为cloud_filtered
   
    double sumx = 0.0, sumy = 0.0, sumz = 0.0;
    for (auto & iter_point : cloud_filtered->points)
    {
        sumx += iter_point.x;
        sumy += iter_point.y;
        sumz += iter_point.z;
    }
    pcl::PointXYZ center(sumx/cloud_filtered->size(), sumy/cloud_filtered->size(), sumz/cloud_filtered->size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr planepoints(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3d normal;
    getPlanePoints(planepoints, normal, cloud_filtered, center);
    Eigen::Vector3d center_eigen(center.x, center.y, center.z);
    vector<Eigen::Vector3d> poly = fitPoly(*planepoints, normal, center_eigen);
    cout<<"get poly"<<endl;
    pcl::PointCloud<pcl::PointXYZRGB> rgbcloud;
    for (auto & iter : planepoints->points)
    {
      pcl::PointXYZRGB p(0, 255, 0);
      p.x = iter.x;
      p.y = iter.y;
      p.z = iter.z;
      rgbcloud.emplace_back(p);
    }
    
    for (auto & point : poly)
    {
      pcl::PointXYZRGB p(255, 0, 0);
      p.x = point(0);
      p.y = point(1);
      p.z = point(2);
      rgbcloud.emplace_back(p);
    }
    cout<<"get rgb cloud"<<endl;
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(rgbcloud.makeShared());
    system("read -p 'Press Enter to continue...' var");
    return 0;
}