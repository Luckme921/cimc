#include <algorithm>
#include <array>
#include <cmath>

#include <fstream>
#include <iostream>
#include <random>
#include <limits>
#include <vector>

#include "gooseneck_trajectory.hpp"
// Row 固定对应 PLY 的 XYZ 和 NX/NY/NZ。提取判定使用 XYZ，原法向随顶点写回。
using Row=std::array<float,6>;
V xyz(const Row& r){return {r[0],r[1],r[2]};}
std::vector<Row> read(const std::string& path){
 std::ifstream in(path,std::ios::binary);
 std::string line,format;
 size_t count=0;
 std::vector<std::string> properties;
 while(std::getline(in,line)){
  if(!line.empty()&&line.back()=='\r')line.pop_back();
  if(line=="end_header")break;
  if(line.rfind("format ",0)==0)format=line;
  if(line.rfind("element vertex ",0)==0)count=std::stoull(line.substr(15));
  if(line.rfind("property ",0)==0)properties.push_back(line.substr(9));
 }
 if(line!="end_header" || format!="format binary_little_endian 1.0" || !count ||
    properties!=std::vector<std::string>{"float x","float y","float z","float nx","float ny","float nz"})
  throw std::runtime_error("Unsupported PLY layout");
 std::vector<Row> rows(count);
 if(!in.read(reinterpret_cast<char*>(rows.data()),count*sizeof(Row)))throw std::runtime_error("Truncated PLY");
 return rows;
}
// 平面方程 n·p=d，n 为单位法向；count 记录阈值内支持点数。
struct Plane{V n{};double d=0;int count=0;};
Plane fit(const std::vector<V>& p,double tolerance){
 std::mt19937 rng(11);
 Plane best;
 for(int t=0;t<1700;++t){
  V a=p[rng()%p.size()],b=p[rng()%p.size()],c=p[rng()%p.size()];
  V n=(b-a).cross(c-a);
  if(n.norm()<1e-8)continue;
  n=n*(1/n.norm());
  double d=n.dot(a);
  int count=0;
  for(V v:p)count+=std::abs(n.dot(v)-d)<tolerance;
  if(count>best.count)best={n,d,count};
 }
 return best;
}

// 取离散分位数。nth_element 会重排传入向量，无需完整排序。
double quantile(std::vector<double>& values,double fraction){
 auto at=values.begin()+size_t((values.size()-1)*fraction);
 std::nth_element(values.begin(),at,values.end());
 return *at;
}
// 一个 U 单元。left/right 是横向粗范围，leftEdge/rightEdge 是按深度层估计的左右边界。
struct Section{
 double left,right;
 std::vector<double> leftEdge,rightEdge;
};
std::ofstream openColoredPly(const std::string& path,size_t count){
 std::ofstream out(path,std::ios::binary);
 if(!out)throw std::runtime_error("Cannot open output PLY: "+path);
 out<<"ply\nformat binary_little_endian 1.0\nelement vertex "<<count
    <<"\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz"
    <<"\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
 return out;
}

void writeColoredVertex(std::ostream& out,const Row& row,const gooseneck::Color& color){
 out.write(reinterpret_cast<const char*>(row.data()),sizeof(Row));
 out.write(reinterpret_cast<const char*>(color.data()),color.size());
}

struct Extracted {
 std::vector<gooseneck::Trajectory> paths;
 V along{},up{},origin{};
 size_t seamCount=0;
};
Extracted extract(const std::string& path,double verticalLength,const gooseneck::Options* trajectory){
 const auto rows=read(path);
 std::vector<V> sample;
 for(size_t i=0;i<rows.size();i+=19){
  V v=xyz(rows[i]);
  if(std::isfinite(v.norm()))sample.push_back(v);
 }
 // 顺次拟合并移除优势平面。0.9 mm 用于评分，1.3 mm 用于排除已拟合面附近点。
 std::vector<Plane> planes;
 for(int i=0;i<3;++i){
  if(sample.size()<100)throw std::runtime_error("Insufficient plane support");
  auto plane=fit(sample,0.9);
  if(plane.d<0){plane.n=plane.n*(-1);plane.d=-plane.d;}
  planes.push_back(plane);
  sample.erase(std::remove_if(sample.begin(),sample.end(),[&](V v){
   return std::abs(plane.n.dot(v)-plane.d)<1.3;
  }),sample.end());
 }
 if(planes[0].count<100 || planes[1].count<100 || planes[2].count<100 ||
    std::abs(planes[0].n.dot(planes[1].n))<0.97 ||
    std::abs(planes[0].n.dot(planes[2].n))>0.25)
  throw std::runtime_error("Expected two parallel upright faces and a base face");
 const auto wall=planes[0].d>planes[1].d?planes[0]:planes[1];
 const auto front=planes[0].d<planes[1].d?planes[0]:planes[1];
 const auto floor=planes[2];
 V axis=wall.n.cross(floor.n);
 if(axis.norm()<1e-8)throw std::runtime_error("Degenerate reference planes");
 axis=axis*((axis.x<0?-1:1)/axis.norm());
 double heightSum=0;
 std::vector<V> local;
 std::vector<double> along,frontDepth;
 for(auto r:rows){
  V v=xyz(r);
  // q.x 为横向投影，q.y 为背板深度，q.z 为底板有符号距离，后面统一朝上。
  V q{axis.dot(v),wall.d-wall.n.dot(v),floor.n.dot(v)-floor.d};
  local.push_back(q);
  if(!std::isfinite(q.norm()))continue;
  along.push_back(q.x);
  heightSum+=q.z;
  if(std::abs(front.n.dot(v)-front.d)<1.2)frontDepth.push_back(q.y);
 }
 if(frontDepth.empty())throw std::runtime_error("Missing U face");
 const double depth=quantile(frontDepth,0.5);
 if(depth<12 || depth>100)throw std::runtime_error("Unsupported U projection depth");
 const double start=quantile(along,0.001)-4;
 const int width=int((quantile(along,0.999)-start)/2)+5;
 if(width<10 || width>100000)throw std::runtime_error("Unsupported ROI span");
 for(V& q:local)q.z*=heightSum<0?-1:1;
 auto column=[&](double x){return int(std::floor((x-start)/2));};
 // 每个 2 mm 横向分箱统计高处前脸点，连续有支撑的列构成一个 U 区间。
 std::vector<int> face(width);
 for(V q:local){
  if(!std::isfinite(q.norm()))continue;
  int x=column(q.x);
  if(x>=0&&x<width&&q.z>25&&std::abs(q.y-depth)<4)++face[x];
 }
 const int support=std::max(4,*std::max_element(face.begin(),face.end())/12);
 std::vector<Section> sections;
 const int layers=int(std::ceil(depth/2))+1;
 for(int x=0;x<width;){
  if(face[x]<support){++x;continue;}
  int begin=x++;
  while(x<width && (face[x]>=support || (x+1<width&&face[x+1]>=support)))++x;
  if(x-begin<12)continue;
  Section section{start+2*begin,start+2*x,{},{}};
  // 按 2 mm 深度分层，从粗边界附近估计细边界。层内支持不足时沿用上一层。
  for(int k=0;k<layers;++k){
   const double y=std::min(depth-3.0,2.0*k);
   std::vector<double> left,right;
   for(V q:local){
    if(q.z<24||std::abs(q.y-y)>1.7)continue;
    if(std::abs(q.x-section.left)<11)left.push_back(q.x);
    if(std::abs(q.x-section.right)<11)right.push_back(q.x);
   }
   const double previousLeft=k?section.leftEdge.back():section.left;
   const double previousRight=k?section.rightEdge.back():section.right;
   section.leftEdge.push_back(left.size()>10?quantile(left,k==0?0.85:0.5):previousLeft);
   section.rightEdge.push_back(right.size()>10?quantile(right,k==0?0.15:0.5):previousRight);
  }
  sections.push_back(std::move(section));
 }
 if(sections.empty())throw std::runtime_error("No sustained U face detected");

 // 从前脸之外的外露底板估算 base[x]。邻列中值吸收局部高度变化，NaN 保留缺测。
 std::vector<std::vector<double>> baseSamples(width);
 for(V q:local){
  if(!std::isfinite(q.norm()))continue;
  int x=column(q.x);
  if(x>=0&&x<width&&q.y>depth+7&&std::abs(q.z)<12)
   baseSamples[x].push_back(q.z);
 }
 const double missing=std::numeric_limits<double>::quiet_NaN();
 std::vector<double> base(width,missing);
 for(int x=0;x<width;++x){
  std::vector<double> nearby;
  for(int j=std::max(0,x-3);j<=std::min(width-1,x+3);++j)
   nearby.insert(nearby.end(),baseSamples[j].begin(),baseSamples[j].end());
  if(nearby.size()>=12)base[x]=quantile(nearby,0.5);
 }

 std::vector<std::vector<double>> bendSamples(layers);
 for(V q:local){
  if(!std::isfinite(q.norm()))continue;
  int x=column(q.x);
  if(x<0||x>=width||!std::isfinite(base[x])||q.y<1||q.y>depth||std::abs(q.z-base[x])>18)continue;
  bool exposed=true;
  for(const Section& section:sections)
   if(q.x>section.left-12&&q.x<section.right+12)exposed=false;
  if(exposed)bendSamples[std::min(layers-1,int(q.y/2))].push_back(q.z-base[x]);
 }
 // bend[k] 用外露区域相对 base 的低分位高度表示 L 圆角抬升，按深度层索引。
 std::vector<double> bend(layers);
 for(int k=layers-1;k>=0;--k){
  if(bendSamples[k].size()>=12)
   bend[k]=std::max(0.0,quantile(bendSamples[k],0.25));
  else if(k+1<layers)bend[k]=bend[k+1];
 }
 const double heightStep=4.0;
 const int heightBins=int(std::ceil(verticalLength/heightStep))+1;
 // 三个深度带分别收集近侧壁、远侧壁和背板的横向坐标，用来估计竖向交界。
 using EdgeSamples=std::array<std::vector<double>,3>;
 std::vector<std::vector<EdgeSamples>> samples(2*sections.size(),std::vector<EdgeSamples>(heightBins));
 for(V q:local){
  if(!std::isfinite(q.norm())||q.y< -1.8||q.y>=14||(q.y>1.2&&q.y<4))continue;
  const double h=q.z-base[std::clamp(column(q.x),0,width-1)];
  if(!std::isfinite(h)||h<6||h>verticalLength+8)continue;
  const int band=q.y<=1.2?2:(q.y<9?0:1);
  for(size_t edge=0;edge<samples.size();++edge){
   const auto& section=sections[edge/2];
   const int layer=std::clamp(int(q.y/2),0,layers-1);
   const double coarse=edge%2?section.rightEdge[layer]:section.leftEdge[layer];
   if(std::abs(q.x-coarse)>9)continue;
   for(int k=std::max(0,int((h-6)/heightStep)-2);k<=std::min(heightBins-1,int((h+6)/heightStep));++k){
    const double center=std::max(8.0,k*heightStep);
    if(std::abs(h-center)<=6)samples[edge][k][band].push_back(q.x);
   }
  }
 }
 // contacts[侧边][高度层] 保存接触线 X。背板分位数可结合侧壁外推，缺支撑时保持 NaN。
 std::vector<std::vector<double>> contacts(samples.size(),std::vector<double>(heightBins,missing));
 for(size_t edge=0;edge<samples.size();++edge){
  for(int k=0;k<heightBins;++k){
   auto& bands=samples[edge][k];
   if(bands[2].size()>=12)
    contacts[edge][k]=quantile(bands[2],edge%2?0.04:0.96);
   if(bands[0].size()<8||bands[1].size()<8)continue;
   const double near=quantile(bands[0],0.5);
   const double far=quantile(bands[1],0.5);
      if(std::abs(near-far)<=3){
    const double sideContact=near-(far-near)*6.5/5.0;
    if(!std::isfinite(contacts[edge][k]))contacts[edge][k]=sideContact;
    else if(std::abs(contacts[edge][k]-sideContact)<=3)
     contacts[edge][k]=(contacts[edge][k]+sideContact)/2;
   }
  }
 }
 std::vector<std::vector<size_t>> coverage(samples.size(),std::vector<size_t>(heightBins));
 double highestVertical=0;
 const std::array<gooseneck::Color,9> palette{{
  {115,120,130},{60,145,245},{55,195,115},{155,105,225},
  {245,205,55},{255,150,55},{55,205,220},{235,120,195},{255,0,0}
 }};
 const auto stem=path.substr(0,path.size()-4);
 auto out=openColoredPly(stem+"_colored.ply",rows.size());
 std::vector<unsigned char> labels(trajectory?rows.size():0);
 std::array<size_t,9> counts{};
 size_t vertical=0,bottom=0;
 std::vector<gooseneck::Samples> seamSamples(trajectory?sections.size():0);
 for(size_t i=0;i<rows.size();++i){
  const V q=local[i];
  int label=0;
  if(std::isfinite(q.norm())){
   int x=std::clamp(column(q.x),0,width-1);
   const double h=q.z-base[x];
   const int layer=std::clamp(int(q.y/2),0,layers-1);
   if(std::abs(q.y)<3.5)label=1;
   if(std::abs(h)<3.5&&q.y>5)label=2;
   if(q.y>1.5&&q.y<12&&h>2&&h<16)label=3;
   for(size_t index=0;index<sections.size();++index){
    const Section& section=sections[index];
    const double left=section.leftEdge[layer],right=section.rightEdge[layer];
    if(q.x<left-5||q.x>right+5)continue;
    const double side=std::min(std::abs(q.x-left),std::abs(q.x-right));
    const bool onFront=q.y>depth-3.2&&q.y<depth+3.5;
    const bool onSide=q.y>1.5&&q.y<depth-2&&side<4;
    if((h>2||!std::isfinite(h))&&onFront)label=4;
    if((h>2||!std::isfinite(h))&&onSide)label=q.x<(left+right)/2?5:6;
    if(label>=4&&label<=6&&side<9&&q.y>depth-8&&q.y<depth-1.5)label=7;
    // 先认竖缝。只有相邻两高度层都有接触位置，才插值得到该高度的 X 并应用 2.2 mm 容差。
    int matchedEdge=-1;
    if(std::isfinite(h)&&h>=std::max(0.0,bend[layer]-1)&&h<=verticalLength&&q.y>-1.8&&q.y<3.8){
     const double bin=h/heightStep;
     const int k=std::min(heightBins-2,int(bin));
     for(int sideIndex=0;sideIndex<2;++sideIndex){
      const auto& trace=contacts[index*2+sideIndex];
      if(!std::isfinite(trace[k])||!std::isfinite(trace[k+1]))continue;
      const double edge=trace[k]+(trace[k+1]-trace[k])*(bin-k);
      if(std::abs(q.x-edge)<=2.2)matchedEdge=int(index*2+sideIndex);
     }
    }
    // 底缝同时检查侧边或前脸的位置，以及相对底板的高度是否靠近 bend。
    const bool bottomJoint=q.y>1.5&&q.y<depth+3.5&&
      std::abs(h-bend[layer])<2.6&&(onFront||side<3.0);
    if(matchedEdge>=0){
     ++coverage[matchedEdge][int(h/heightStep)];
     highestVertical=std::max(highestVertical,h);
     label=8;
     ++vertical;
     if(trajectory)seamSamples[index][matchedEdge%2].push_back({xyz(rows[i]),q,h});
    }else if(bottomJoint){
     label=8;
     ++bottom;
     if(trajectory)seamSamples[index][2].push_back({xyz(rows[i]),q,h});
    }
    if(label>=4)break;
   }
  }
  ++counts[label];
  writeColoredVertex(out,rows[i],palette[label]);
  if(trajectory)labels[i]=static_cast<unsigned char>(label);
 }
 out.close();
 if(!out)throw std::runtime_error("Could not write colored PLY");
 // 可选 TCP 分支直接消费本次按 U 编组的红色样本，不从已写出的彩色文件读回。
 Extracted extracted;
 extracted.along=axis;
 extracted.up=floor.n*(heightSum<0?-1:1);
 const double normalDot=wall.n.dot(floor.n);
 extracted.origin=wall.n*((wall.d-normalDot*floor.d)/(1-normalDot*normalDot))+
                  floor.n*((floor.d-normalDot*wall.d)/(1-normalDot*normalDot));
 extracted.seamCount=vertical+bottom;
 if(trajectory){
  std::vector<gooseneck::Trajectory> paths;
  for(size_t i=0;i<sections.size();++i){
   const auto& section=sections[i];
   try{
    paths.push_back(gooseneck::makeTrajectory(seamSamples[i],section.leftEdge,section.rightEdge,
     bend,depth,verticalLength,axis,wall.n*(-1),front.n*(front.n.dot(wall.n)>0?-1:1),
     floor.n*(heightSum<0?-1:1),*trajectory));
   }catch(const std::exception& error){
    throw std::runtime_error("U "+std::to_string(i+1)+": "+error.what());
   }
  }
  std::vector<gooseneck::PreviewPoint> markers;
  for(size_t i=0;i<paths.size();++i){
   const auto name=stem+"_U"+std::to_string(i+1)+"_trajectory";
   gooseneck::writeTrajectoryCsv(name,paths[i]);
   gooseneck::appendTrajectoryPreview(markers,paths[i]);
   std::cout<<name<<": 8 poses\n";
  }
  const auto resultPath=stem+"_weld_result.ply";
  auto combined=openColoredPly(resultPath,rows.size()+markers.size());
  for(size_t i=0;i<rows.size();++i)writeColoredVertex(combined,rows[i],palette[labels[i]]);
  for(const auto& marker:markers){
   const V p=marker.position;
   writeColoredVertex(combined,{float(p.x),float(p.y),float(p.z),0,0,0},marker.color);
  }
  combined.close();
  if(!combined)throw std::runtime_error("Cannot write combined PLY: "+resultPath);
  std::cout<<resultPath<<": "<<paths.size()*8<<" poses overlaid on segmented cloud\n";
  extracted.paths=std::move(paths);
 }
 std::cout<<path<<": U plates="<<sections.size()<<", depth="<<depth
          <<", vertical limit="<<verticalLength<<", highest vertical="<<highestVertical
          <<", seam vertical="<<vertical<<", seam bottom="<<bottom<<"\n";
 for(const auto& section:sections)
  std::cout<<"  U span "<<section.left<<" .. "<<section.right<<"\n";
 for(size_t edge=0;edge<coverage.size();++edge){
  std::cout<<"  U "<<edge/2+1<<(edge%2?" right":" left")<<" vertical bins:";
  for(int k=0;k*heightStep<verticalLength;++k)std::cout<<" "<<coverage[edge][k];
  std::cout<<"\n";
 }
 std::cout<<"  Label counts:";
 for(size_t count:counts)std::cout<<" "<<count;
 std::cout<<"\n";
 return extracted;
}
#ifndef GOOSENECK_NO_MAIN
int main(int argc,char** argv){
 if(argc<2){
  std::cerr<<"Usage: weld_extract [--vertical-length-mm N] [--trajectory] [--clearance-mm N] [--tool-z-to-body] [--offsets FILE] input.ply ...\n";
  return 2;
 }
 // 人为指定的竖缝提取高度上限，单位 mm，可由命令行覆盖。
 double verticalLength=40;
 gooseneck::Options options;
 bool generateTrajectory=false;
 int firstInput=1;
 try{
  auto number=[](const std::string& value){
   size_t consumed=0;
   const double result=std::stod(value,&consumed);
   if(consumed!=value.size()||!std::isfinite(result))throw std::invalid_argument("Expected a finite number");
   return result;
  };
  while(firstInput<argc&&std::string(argv[firstInput]).rfind("--",0)==0){
   const std::string flag=argv[firstInput++];
   if(flag=="--trajectory"){generateTrajectory=true;continue;}
   if(flag=="--tool-z-to-body"){options.positiveBodyZ=true;continue;}
   if(firstInput>=argc)throw std::invalid_argument("Missing value for "+flag);
   const std::string value=argv[firstInput++];
   if(flag=="--vertical-length-mm"){
    verticalLength=number(value);
    if(verticalLength<1||verticalLength>10000)throw std::invalid_argument("Vertical length must be 1 to 10000 mm");
   }else if(flag=="--clearance-mm"){
    options.clearance=number(value);
    if(options.clearance<=0)throw std::invalid_argument("Clearance must be positive");
   }else if(flag=="--offsets"){
    std::ifstream offsets(value);
    if(!offsets)throw std::invalid_argument("Cannot open offsets file");
    std::array<bool,8> used{};
    std::string line;
    while(std::getline(offsets,line)){
     std::istringstream row(line);
     std::string index;
     if(!(row>>index)||index[0]=='#')continue;
     const double i=number(index);
     if(i<0||i>7||std::floor(i)!=i||used[int(i)])throw std::invalid_argument("Invalid or duplicate offset order");
     used[int(i)]=true;
     std::array<double,6> fields{};
     for(double& field:fields){
      std::string token;
      if(!(row>>token))throw std::invalid_argument("Expected order dx dy dz roll pitch yaw");
      field=number(token);
     }
     if(row>>index)throw std::invalid_argument("Unexpected offset field");
     options.offsets[int(i)]={{fields[0],fields[1],fields[2]},{fields[3],fields[4],fields[5]}};
    }
   }else throw std::invalid_argument("Unknown option: "+flag);
  }
  if(firstInput==argc)throw std::invalid_argument("Missing input PLY");
 }catch(const std::exception& error){
  std::cerr<<error.what()<<"\n";
  return 2;
 }
 int status=0;
 for(int i=firstInput;i<argc;++i){
  try{
   std::string path=argv[i];
   if(path.size()<5||path.substr(path.size()-4)!=".ply")
    throw std::runtime_error("Expected .ply input");
   extract(path,verticalLength,generateTrajectory?&options:nullptr);
  }catch(const std::exception& error){
   std::cerr<<argv[i]<<": "<<error.what()<<"\n";
   status=1;
  }
 }
 return status;
}
#endif
