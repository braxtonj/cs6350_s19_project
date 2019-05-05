#include <utils.h>



int main(int argc, char** argv)
{
  PtXYZ pt1,pt2;
  pt1.x = 1.0;
  pt1.y = 1.0;
  pt1.z = 1.0;
  
  pt2.x = 4.0;
  pt2.y = 1.0;
  pt2.z = 1.0;

  using namespace PCSeg::utils;
  
  std::cout << "Dist:" << dist(pt1,pt2) << std::endl;
  // correct answer is 3.0

  std::cout << "Sqr Dist:" << sqrDist(pt1,pt2) << std::endl;
  // correct answer is 9.0

}
