#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

struct V{
 double x,y,z;
 V operator+(V b)const{return {x+b.x,y+b.y,z+b.z};}
 V operator-(V b)const{return {x-b.x,y-b.y,z-b.z};}
 V operator*(double s)const{return {x*s,y*s,z*s};}
 double dot(V b)const{return x*b.x+y*b.y+z*b.z;}
 V cross(V b)const{return {y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x};}
 double norm()const{return std::sqrt(dot(*this));}
};

namespace gooseneck {

// position 为相机轴 XYZ 偏移（mm），angles 为依次绕当前工具 X/Y/Z 的角度偏移（度）。
struct Offset {
 V position{};
 V angles{};
};

struct Options {
 double clearance = 30;
 bool jointBisectors = false;
 // Signed correction from the original bisector: positive tilts physical body
 // toward the base normal, independent of the chosen tool X/Z signs.
 double bodyTiltDegrees = 0;
 // Positive leans away from the left/right upright, respectively. Tool X is
 // the closest orthogonal projection of the original reference, never flipped.
 double sideLeanDegrees = 0;
 bool positiveBodyZ = false;
 bool positiveToolX = true;
 std::array<Offset,8> offsets{};
};

// 同时保留相机坐标 world、工件投影 local 和修正后的离底板高度 height。
struct Sample {
 V world;
 V local;
 double height;
};

using Samples=std::array<std::vector<Sample>,3>;
using Quaternion=std::array<double,4>;

inline V unit(V v){
 const double length=v.norm();
 if(!std::isfinite(length)||length<1e-8)
  throw std::runtime_error("Degenerate trajectory geometry");
 return v*(1/length);
}

inline V rotate(V v,V axis,double degrees){
 const double angle=degrees*std::acos(-1.0)/180;
 return v*std::cos(angle)+axis.cross(v)*std::sin(angle)
       +axis*(axis.dot(v)*(1-std::cos(angle)));
}

inline Quaternion quaternion(V x,V y,V z){
 const double m[3][3]={{x.x,y.x,z.x},{x.y,y.y,z.y},{x.z,y.z,z.z}};
 const double trace=x.x+y.y+z.z;
 Quaternion q{};
 if(trace>0){
  q[0]=0.5*std::sqrt(1+trace);
  for(int i=0;i<3;++i)
   q[i+1]=(m[(i+2)%3][(i+1)%3]-m[(i+1)%3][(i+2)%3])/(4*q[0]);
 }else{
  int i=0;
  for(int j=1;j<3;++j)if(m[j][j]>m[i][i])i=j;
  const int j=(i+1)%3,k=(i+2)%3;
  q[i+1]=0.5*std::sqrt(1+m[i][i]-m[j][j]-m[k][k]);
  q[0]=(m[k][j]-m[j][k])/(4*q[i+1]);
  q[j+1]=(m[j][i]+m[i][j])/(4*q[i+1]);
  q[k+1]=(m[k][i]+m[i][k])/(4*q[i+1]);
 }
 double squared=0;
 for(double v:q)squared+=v*v;
 for(double& v:q)v/=std::sqrt(squared);
 return q;
}

// 在对应焊缝样本组中寻找实测点。比较加权横向距离、深度差与修正高度差。
inline V nearest(const std::vector<Sample>& points,double x,double y,double height){
 const Sample* best=nullptr;
 double bestError=std::numeric_limits<double>::infinity();
 for(const auto& p:points){
  const V delta{(p.local.x-x)*0.9,p.local.y-y,p.height-height};
  const double error=delta.norm();
  if(error<bestError){best=&p;bestError=error;}
 }
 if(!best||bestError>9)throw std::runtime_error("Insufficient seam support near requested waypoint");
 return best->world;
}

struct Trajectory {
 std::array<V,8> position{},nominal{},body{},toolX{},toolY{},toolZ{};
 std::array<V,8> sideNormal{};
 std::array<V,8> jointReferenceNormal{};
 std::array<bool,8> sideNormalMeasured{};
 std::array<int,8> jointSurfaceCount{};
 std::array<Quaternion,8> orientation{};
};

struct SidePlanes {
 std::array<V,2> outward{};
 std::array<bool,2> measured{};
};

inline Trajectory makeTrajectory(const Samples& samples,
 const std::vector<double>& left,const std::vector<double>& right,
 const std::vector<double>& bend,double depth,double height,
 V along,V wall,V front,V base,const Options& options,const SidePlanes& sides={}){
 if(samples[0].empty()||samples[1].empty()||samples[2].empty())
  throw std::runtime_error("Missing vertical or bottom seam samples");
 const auto top=[](const std::vector<Sample>& points){
  return std::max_element(points.begin(),points.end(),[](const Sample& a,const Sample& b){
   return a.height<b.height;
  })->local.x;
 };
 Trajectory path;
 auto& p=path.nominal;
 // 索引 1 和 6 是两端竖缝，2 到 5 是底部四个转角，0 和 7 留给退让点。
 p[1]=nearest(samples[0],top(samples[0]),0,height);
 p[6]=nearest(samples[1],top(samples[1]),0,height);
 if(left.size()<3 || right.size()!=left.size() || bend.size()!=left.size())
  throw std::runtime_error("Insufficient depth layers");
 const int frontLayer=std::clamp(int((depth-3)/2),2,int(left.size())-1);
 p[2]=nearest(samples[2],left[2],4,bend[2]);
 p[3]=nearest(samples[2],left[frontLayer]+2,depth-2,bend[frontLayer]);
 p[4]=nearest(samples[2],right[frontLayer]-2,depth-2,bend[frontLayer]);
 p[5]=nearest(samples[2],right[2],4,bend[2]);
 for(int i=2;i<=6;++i)
  if((p[i]-p[i-1]).norm()<1)throw std::runtime_error("Coincident weld waypoints");

 // bodySign 将工具 +Z 转为物理枪体方向；默认 +Z 从枪体指向 TCP。
 const double bodySign=options.positiveBodyZ?1:-1;
 const V transverse=unit(p[4]-p[3]);
 const V seamX=transverse*(options.positiveToolX?1:-1);
 // Take the bisector in the cross section perpendicular to the transverse seam.
 const V baseNormal=unit(base-seamX*base.dot(seamX));
 const V frontNormal=unit(front-seamX*front.dot(seamX));
 V sharedBody=unit(baseNormal+frontNormal);
 if(!std::isfinite(options.bodyTiltDegrees)||std::abs(options.bodyTiltDegrees)>30)
  throw std::invalid_argument("Body tilt must be finite and within -30..30 degrees");
 if(!std::isfinite(options.sideLeanDegrees)||options.sideLeanDegrees<0||options.sideLeanDegrees>20)
  throw std::invalid_argument("Side lean must be finite and within 0..20 degrees");
 if(options.bodyTiltDegrees!=0)
  sharedBody=unit(rotate(sharedBody,unit(sharedBody.cross(baseNormal)),options.bodyTiltDegrees));
 const V sharedZ=sharedBody*bodySign;
 const std::array<V,3> sharedFrame{seamX,unit(sharedZ.cross(seamX)),sharedZ};
 auto applyAngles=[](std::array<V,3> frame,V angles){
  auto& [x,y,z]=frame;
  y=rotate(y,x,angles.x);
  z=rotate(z,x,angles.x);
  x=rotate(x,y,angles.y);
  z=rotate(z,y,angles.y);
  x=rotate(x,z,angles.z);
  y=rotate(y,z,angles.z);
  return frame;
 };

 // Safety point translation follows the open side of each upright joint.
 auto retreatDirection=[&](V jointNormal,V travel,V angles){
  const V z=unit(jointNormal)*bodySign;
  const V x=unit(travel-z*z.dot(travel));
  return applyAngles({x,unit(z.cross(x)),z},angles)[2]*bodySign;
 };
 const V entryRetreat=retreatDirection(wall-along,p[2]-p[1],options.offsets[1].angles);
 const V exitRetreat=retreatDirection(wall+along,p[6]-p[5],options.offsets[6].angles);
 for(int i=0;i<8;++i){
  auto frame=sharedFrame;
  const int side=i<=3?0:1;
  const V outward=sides.outward[side].norm()>0.5?unit(sides.outward[side]):unit(along)*(side==0?-1:1);
  path.sideNormal[i]=outward;
  path.sideNormalMeasured[i]=sides.measured[side];
  if(options.jointBisectors){
   // Vertical joints: exact bisector of two outward face normals. Bottom
   // corners meet three faces: equal-normal trihedral clearance direction.
   const bool vertical=i<=1||i>=6;
   path.jointReferenceNormal[i]=unit(vertical?wall:(i==2||i==5?wall:front));
   V body=unit(outward+path.jointReferenceNormal[i]+
               (vertical?V{0,0,0}:unit(base)));
   path.jointSurfaceCount[i]=vertical?2:3;
   if(options.bodyTiltDegrees!=0)
    body=unit(rotate(body,unit(body.cross(unit(base))),options.bodyTiltDegrees));
   if(options.sideLeanDegrees!=0)
    body=unit(rotate(body,unit(body.cross(outward)),options.sideLeanDegrees));
   const V z=body*bodySign;
   const V x=unit(seamX-z*seamX.dot(z));
   frame={x,unit(z.cross(x)),z};
  }else if(options.sideLeanDegrees!=0){
   const double a=options.sideLeanDegrees*std::acos(-1.0)/180;
   const V body=unit(sharedBody*std::cos(a)+transverse*((i<=3?-1:1)*std::sin(a)));
   const V z=body*bodySign;
   const V x=unit(seamX-z*seamX.dot(z));
   frame={x,unit(z.cross(x)),z};
  }
  const auto [x,y,z]=applyAngles(frame,options.offsets[i].angles);
  path.toolX[i]=x;
  path.toolY[i]=y;
  path.toolZ[i]=z;
  path.body[i]=z*bodySign;
  path.orientation[i]=quaternion(x,y,z);
  if(i){
   double dot=0;
   for(int j=0;j<4;++j)dot+=path.orientation[i-1][j]*path.orientation[i][j];
   if(dot<0)for(double& v:path.orientation[i])v=-v;
  }
 }
 for(int i=1;i<=6;++i)path.position[i]=p[i]+options.offsets[i].position;
 // 先按竖缝开放侧计算退让点。实际输出继承相邻焊点位置补偿，再叠加本点偏移。
 p[0]=p[1]+entryRetreat*options.clearance;
 p[7]=p[6]+exitRetreat*options.clearance;
 path.position[0]=path.position[1]+entryRetreat*options.clearance+options.offsets[0].position;
 path.position[7]=path.position[6]+exitRetreat*options.clearance+options.offsets[7].position;
 return path;
}

inline void writeTrajectoryCsv(const std::string& stem,const Trajectory& path){
 // weld_enabled describes a weld waypoint, not an arc-start/arc-stop command.
 const char* roles[]={"safe_start","weld_start","corner_1","corner_2","corner_3","corner_4","weld_end","safe_end"};
 std::ofstream csv(stem+".csv");
 csv.imbue(std::locale::classic());
 csv<<"x,y,z,qw,qx,qy,qz,order,kind,weld_enabled,raw_x,raw_y,raw_z,torch_body_axis_x,torch_body_axis_y,torch_body_axis_z\n"
    <<std::setprecision(12);
 for(int i=0;i<8;++i){
  const V p=path.position[i],raw=path.nominal[i],body=path.body[i];
  csv<<p.x<<','<<p.y<<','<<p.z;
  for(double q:path.orientation[i])csv<<','<<q;
  csv<<','<<i<<','<<roles[i]<<','<<(i>=1&&i<=6)
     <<','<<raw.x<<','<<raw.y<<','<<raw.z<<','<<body.x<<','<<body.y<<','<<body.z<<'\n';
 }
 csv.close();
 if(!csv)throw std::runtime_error("Cannot write trajectory CSV");
}

using Color=std::array<unsigned char,3>;

struct PreviewPoint {
 V position;
 Color color;
};

inline void appendTrajectoryPreview(std::vector<PreviewPoint>& points,const Trajectory& path){
 constexpr double trajectoryRadius=0.18;
 constexpr double axisRadius=0.22;
 constexpr double markerWidthRadius=0.15;
 auto line=[&](V a,V b,Color color,double radius){
  constexpr double spacing=0.18;
  constexpr double goldenAngle=2.399963229728653;
  const V delta=b-a;
  const double length=delta.norm();
  if(length<1e-8){
   points.push_back({a,color});
   return;
  }
  const V direction=delta*(1/length);
  const V reference=std::abs(direction.z)<0.9?V{0,0,1}:V{1,0,0};
  const V u=unit(direction.cross(reference));
  const V v=direction.cross(u);
  const int steps=2*int(std::clamp(std::ceil(length/(2*spacing)),1.0,5000.0));
  for(int i=0;i<=steps;++i){
   V point=a+delta*(double(i)/steps);
   // Keep centerline samples and stagger the additional points into one narrow stroke.
   if(i%2){
    const int sample=i/2;
    const double angle=sample*goldenAngle;
    const double distance=radius*std::sqrt(((sample*37)%101+0.5)/101.0);
    point=point+(u*std::cos(angle)+v*std::sin(angle))*distance;
   }
   points.push_back({point,color});
  }
 };
 constexpr Color weldLine{255,105,180};
 constexpr Color safeLine{120,210,255};
 constexpr Color torchAxis{255,255,255};
 constexpr Color toolXAxis{180,255,0};
 for(size_t i=1;i<path.position.size();++i){
  const bool safeMove=i==1||i+1==path.position.size();
  line(path.position[i-1],path.position[i],safeMove?safeLine:weldLine,trajectoryRadius);
 }
 for(size_t i=0;i<path.position.size();++i){
  const V p=path.position[i];
  const bool safePoint=i==0||i+1==path.position.size();
  const Color marker=safePoint?Color{75,230,255}:Color{255,20,147};
  constexpr double markerReach=3;
  for(V axis:{path.toolX[i],path.toolY[i],path.toolZ[i]})
   line(p-axis*markerReach,p+axis*markerReach,marker,markerWidthRadius);
  line(p+path.body[i]*0.5,p+path.body[i]*12,torchAxis,axisRadius);
  const V arrowTip=p+path.toolX[i]*16;
  const V arrowBase=arrowTip-path.toolX[i]*4;
  const V wing=path.toolY[i]*2;
  line(p+path.toolX[i]*0.5,arrowTip,toolXAxis,axisRadius);
  line(arrowTip,arrowBase+wing,toolXAxis,axisRadius);
  line(arrowTip,arrowBase-wing,toolXAxis,axisRadius);
 }
}

}  // namespace gooseneck
