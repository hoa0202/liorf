#include <common_lib.h>
#include <utility.h> // SensorType 정의를 위해 추가

// common_lib_ 전역 변수 정의
std::shared_ptr<CommonLib::common_lib> common_lib_ = std::make_shared<CommonLib::common_lib>("LIORF-ROS2");

namespace CommonLib
{
  template float common_lib::pointDistance<pcl::PointXYZI>(const pcl::PointXYZI& p);
  template float common_lib::pointDistance<pcl::PointXYZINormal>(const pcl::PointXYZINormal& p);

  template float common_lib::pointDistance<pcl::PointXYZI>(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2);
  template float common_lib::pointDistance<pcl::PointXYZINormal>(const pcl::PointXYZINormal& p1, const pcl::PointXYZINormal& p2);

  common_lib::common_lib(const std::string& pkg_mode_)
  {
    PKG_VERSION(pkg_mode_);
  }

  common_lib::~common_lib() {}

  void common_lib::PKG_VERSION(const std::string& pkg_mode_)
  {
    std::cout << "\033[34m" << std::string(60, '*')                       << std::endl;
    std::cout << "Author          : yjz_lucky_boy"                        << std::endl;
    std::cout << "Slam mode       : " << pkg_mode_                        << std::endl;
    std::cout << "Package version : v0.0.0"                               << std::endl;
    std::cout << "Package link    : https://github.com/YJZLuckyBoy/liorf" << std::endl;
    std::cout << std::string(60, '*') << "\033[0m"                        << std::endl;
  }

  template<typename T> 
  float common_lib::pointDistance(const T& p)
  { 
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  }

  template<typename T>
  float common_lib::pointDistance(const T& p1, const T& p2)
  {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
  }

  template<typename T> 
  bool common_lib::remapping(const T& value, const T& graph)
  {
    std::cout << "waiting ........" << std::endl;
    return -1;
  }
} // namespace CommonLib