#include <windows.h>
#include <vector>
#include <string>
 
using namespace std;
 
//////////////////////////////////////////////////////
struct Point {
  double x, y;
};
 
static double DistanceSqrd(const Point& point, double x, double y) {
  double xd = x - point.x;
  double yd = y - point.y;
  return (xd * xd) + (yd * yd);
}
 
//////////////////////////////////////////////////////
class Voronoi {
 public:
  void Make(int count) {
    CreatePoints(count);
    CreateSites();
    SetSitesPoints();
  }
 
 private:
  void CreateSites() {
    int w = bmp_->width(), h = bmp_->height(), d;
    for (int hh = 0; hh < h; hh++) {
      for (int ww = 0; ww < w; ww++) {
        int ind = -1, dist = INT_MAX;
        for (size_t it = 0; it < points_.size(); it++) {
          const Point& p = points_[it];
          d = DistanceSqrd(p, ww, hh);
          if (d < dist) {
            dist = d;
            ind = it;
          }
        }
 
        if (ind > -1)
          SetPixel(bmp_->hdc(), ww, hh, colors_[ind]);
        else
          __asm nop // should never happen!
        }
    }
  }
 
  void SetSitesPoints() {
    for (const auto& point : points_) {
      int x = point.x, y = point.y;
      for (int i = -1; i < 2; i++)
        for (int j = -1; j < 2; j++)
          SetPixel(bmp_->hdc(), x + i, y + j, 0);
    }
  }
 
  void CreatePoints(int count) {
    const int w = bmp_->width() - 20, h = bmp_->height() - 20;
    for (int i = 0; i < count; i++) {
      points_.push_back({ rand() % w + 10, rand() % h + 10 });
    }
  }
 
  void CreateColors() {
    for (size_t i = 0; i < points_.size(); i++) {
      DWORD c = RGB(rand() % 200 + 50, rand() % 200 + 55, rand() % 200 + 50);
      colors_.push_back(c);
    }
  }
 
  vector<Point> points_;
  vector<DWORD> colors_;
  MyBitmap* bmp_;
};
 
//////////////////////////////////////////////////////
int main(int argc, char* argv[]) {
  ShowWindow(GetConsoleWindow(), SW_MAXIMIZE);
  srand(GetTickCount());
 
  MyBitmap bmp;
  bmp.Create(512, 512);
  bmp.SetPenColor(0);
 
  Voronoi v;
  v.Make(&bmp, 50);
 
  BitBlt(GetDC(GetConsoleWindow()), 20, 20, 512, 512, bmp.hdc(), 0, 0, SRCCOPY);
  bmp.SaveBitmap("v.bmp");
 
  system("pause");
 
  return 0;
}