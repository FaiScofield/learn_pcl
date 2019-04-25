/* \author Kripasindhu Sarkar */

#include <iostream>
#include <map>
#include <vector>
#include <pcl/visualization/pcl_painter2D.h>
//----------------------------------------------------------------------------

int main (int argc, char * argv [])
{
  pcl::visualization::PCLPainter2D *painter = new pcl::visualization::PCLPainter2D();

  int winw = 800, winh = 600;
  painter->setWindowSize (winw, winh);
  int xpos = 0;
  int r = winw;
  int R = 50;
  int inc = 5;
  int noc = winw/R;

  while (1) {
    //draw noc no of circles
    for (int i = 0; i < noc; i++) {
      if (i % 2)
        painter->setBrushColor(0, 0, 0, 200);
      else
        painter->setBrushColor(255, 255, 255, 200);

      int rad = r - i * R;
      if (rad < 0) { rad = winw + rad;}

      painter->addCircle(winw/2, winh/2, rad); // 图像坐标系原点是从左下角
    }

    r -= inc;
    if (r < winw - R) r = winw + R;

    painter->setBrushColor(255, 0, 0, 100);
    painter->addRect((xpos += inc) % winw, 200, 100, 100); // x,y(左下角坐标),w,d(长宽)

    //display
    painter->spinOnce();
    painter->clearFigures();
  }


  return 0;
}
