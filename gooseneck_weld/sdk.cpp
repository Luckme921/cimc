#include <gooseneck_sdk/gooseneck_sdk.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <chrono>
#include <filesystem>
#include <map>
#include <set>
#include <sstream>
#include <unistd.h>
#define GOOSENECK_NO_MAIN
#include "weld_extract.cpp"

namespace {
namespace fs=std::filesystem;
const std::array<std::string,8> roles{{"safe_start","weld_start","corner_1","corner_2",
 "corner_3","corner_4","weld_end","safe_end"}};
const std::array<std::string,6> fields{{"dx","dy","dz","roll","pitch","yaw"}};
using Settings=std::map<std::string,std::string>;
std::string trim(const std::string& s){
 const auto a=s.find_first_not_of(" \t\r\n");
 return a==s.npos?"":s.substr(a,s.find_last_not_of(" \t\r\n")-a+1);
}
void insert(Settings& s,const std::string& item){
 const auto eq=item.find('=');
 if(eq==item.npos)throw std::invalid_argument("Expected key=value: "+item);
 const auto k=trim(item.substr(0,eq)),v=trim(item.substr(eq+1));
 if(!s.count(k)||v.empty())throw std::invalid_argument("Unknown/empty parameter: "+k);
 s[k]=v;
}
double number(const Settings& s,const std::string& k,bool infinity=false){
 size_t n=0;const double v=std::stod(s.at(k),&n);
 if(n!=s.at(k).size()||std::isnan(v)||(!infinity&&!std::isfinite(v)))
  throw std::invalid_argument("Invalid number: "+k);
 return v;
}
bool boolean(const Settings& s,const std::string& k){
 if(s.at(k)=="true")return true;if(s.at(k)=="false")return false;
 throw std::invalid_argument("Expected true/false: "+k);
}
Settings load(const gooseneck_sdk::RunOptions& o){
 Settings s{{"gooseneck.vertical_length_mm","40"},{"gooseneck.clearance_mm","30"},
 {"gooseneck.tool_x_positive","true"},{"gooseneck.tool_z_to_body","false"},
 {"gooseneck.groove_index","1"},{"gooseneck.body_tilt_deg","0"},
 {"gooseneck.side_lean_deg","0"},{"roi.enable","false"}};
 for(char a:std::string("xyz")){s[std::string("roi.min_")+a]="-inf";s[std::string("roi.max_")+a]="inf";}
 for(const auto& r:roles)for(const auto& f:fields)s["gooseneck."+r+"."+f]="0";
 if(!o.config_file.empty()){
  std::ifstream in(o.config_file);if(!in)throw std::runtime_error("Cannot open config: "+o.config_file);
  std::string line;while(std::getline(in,line)){line=trim(line.substr(0,line.find('#')));if(!line.empty())insert(s,line);}
 }
 std::set<std::string> used;
 for(const auto& item:o.parameter_overrides){const auto k=trim(item.substr(0,item.find('=')));
  if(!used.insert(k).second)throw std::invalid_argument("Duplicate parameter: "+k);insert(s,item);}
 return s;
}
struct Scratch{
 fs::path path;
 explicit Scratch(const fs::path& parent){auto s=(parent/".gooseneck-XXXXXX").string();
  std::vector<char> b(s.begin(),s.end());b.push_back(0);
  if(!mkdtemp(b.data()))throw std::runtime_error("Cannot create temporary output directory");path=b.data();}
 ~Scratch(){std::error_code e;fs::remove_all(path,e);}
};
void stageInput(const fs::path& source,const fs::path& target,const Settings& s){
 pcl::PointCloud<pcl::PointXYZ> cloud;
 if(pcl::io::loadPLYFile(source.string(),cloud)<0)throw std::runtime_error("Cannot read input PLY: "+source.string());
 std::array<double,3> low{},high{};
 for(int i=0;i<3;++i){const std::string a(1,"xyz"[i]);
  low[i]=number(s,"roi.min_"+a,true);high[i]=number(s,"roi.max_"+a,true);
  if(low[i]>high[i])throw std::invalid_argument("ROI minimum exceeds maximum: "+a);}
 const bool roi=boolean(s,"roi.enable");std::vector<Row> rows;
 for(const auto& p:cloud){
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
  if(roi&&(p.x<low[0]||p.x>high[0]||p.y<low[1]||p.y>high[1]||p.z<low[2]||p.z>high[2]))continue;
  // Segmentation consumes XYZ only. Normals are not used by the portable core.
  rows.push_back({p.x,p.y,p.z,0,0,0});}
 if(rows.size()<1000)throw std::runtime_error("ROI has fewer than 1000 valid points; retain base and both upright faces");
 std::ofstream out(target,std::ios::binary);
 out<<"ply\nformat binary_little_endian 1.0\nelement vertex "<<rows.size()
 <<"\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n";
 out.write(reinterpret_cast<const char*>(rows.data()),rows.size()*sizeof(Row));
 out.close();if(!out)throw std::runtime_error("Cannot stage input PLY");
}
void writeRows(const fs::path& target,const gooseneck::Trajectory& p,const Extracted& g,
 const gooseneck::Options& o,int groove){
 std::ofstream csv(target);csv.imbue(std::locale::classic());
 csv<<"order,x,y,z,qw,qx,qy,qz,raw_x,raw_y,raw_z,workpiece_x,workpiece_y,workpiece_z,"
 "raw_workpiece_x,raw_workpiece_y,raw_workpiece_z,pose_group,pose_source,weld_enabled,"
 "position_offset_group,applied_offset_workpiece_x,applied_offset_workpiece_y,applied_offset_workpiece_z,"
 "applied_offset_camera_x,applied_offset_camera_y,applied_offset_camera_z,roll,pitch,yaw,"
 "feature_type,point_source,groove,groove_order,world_tool_x_axis_x,world_tool_x_axis_y,world_tool_x_axis_z,"
 "world_torch_body_axis_x,world_torch_body_axis_y,world_torch_body_axis_z,tool_positive_z_direction,"
 "tool_x_reference,tool_x_workpiece_direction,body_tilt_deg,body_angle_to_base_deg,side_lean_deg\n"<<std::setprecision(12);
 const V x=gooseneck::unit(g.along),z=gooseneck::unit(g.up-x*g.up.dot(x)),y=z.cross(x);
 for(int i=0;i<8;++i){const auto v=p.position[i],raw=p.nominal[i],offset=v-raw,a=o.offsets[i].angles;
  const V local=v-g.origin,rawLocal=raw-g.origin;
  csv<<i<<','<<v.x<<','<<v.y<<','<<v.z;
  for(double q:p.orientation[i])csv<<','<<q;
  csv<<','<<raw.x<<','<<raw.y<<','<<raw.z<<','<<local.dot(x)<<','<<local.dot(y)<<','<<local.dot(z)
   <<','<<rawLocal.dot(x)<<','<<rawLocal.dot(y)<<','<<rawLocal.dot(z)<<','<<roles[i]<<','
   <<(o.sideLeanDegrees!=0?"outward_side_lean_frame":o.bodyTiltDegrees!=0?"tilted_transverse_frame":"shared_transverse_frame")<<','
   <<(i>=1&&i<=6)<<','<<roles[i]<<','<<offset.dot(x)<<','<<offset.dot(y)<<','<<offset.dot(z)
   <<','<<offset.x<<','<<offset.y<<','<<offset.z<<','<<a.x<<','<<a.y<<','<<a.z
   <<','<<roles[i]<<','<<(i==0||i==7?"generated_safe_transition":"measured_red_seam_point")
   <<','<<groove<<','<<i<<','<<p.toolX[i].x<<','<<p.toolX[i].y<<','<<p.toolX[i].z
   <<','<<p.body[i].x<<','<<p.body[i].y<<','<<p.body[i].z
   <<','<<(o.positiveBodyZ?"tcp_to_body":"body_to_tcp")
   <<",gooseneck_transverse,"<<(o.positiveToolX?"positive_x":"negative_x")
   <<','<<o.bodyTiltDegrees<<','<<std::asin(std::clamp(p.body[i].dot(gooseneck::unit(g.up)),-1.0,1.0))*180/std::acos(-1.0)
   <<','<<o.sideLeanDegrees<<'\n';}
 csv.close();if(!csv)throw std::runtime_error("Cannot write normalized CSV");
}
void writeCloud(const fs::path& target,const pcl::PointCloud<pcl::PointXYZRGB>& points){
 std::ofstream out(target,std::ios::binary);
 out<<"ply\nformat binary_little_endian 1.0\nelement vertex "<<points.size()
 <<"\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
 for(const auto& p:points){const float xyz[3]={p.x,p.y,p.z};const unsigned char rgb[3]={p.r,p.g,p.b};
  out.write(reinterpret_cast<const char*>(xyz),sizeof(xyz));out.write(reinterpret_cast<const char*>(rgb),3);}
 out.close();if(!out)throw std::runtime_error("Cannot write result PLY");
}
}

const char* gooseneck_sdk::version(){return "1.0.2";}
gooseneck_sdk::RunResult gooseneck_sdk::run(const RunOptions& options){
 RunResult result;const auto start=std::chrono::steady_clock::now();
 try{
  if(options.output_prefix.empty()||fs::path(options.output_prefix).filename()!=options.output_prefix||
   options.output_prefix=="."||options.output_prefix=="..")throw std::invalid_argument("Output prefix must be a single filename");
  const auto s=load(options);const double height=number(s,"gooseneck.vertical_length_mm"),selected=number(s,"gooseneck.groove_index");
  if(height<1||height>10000||selected<1||selected>100||selected!=std::floor(selected))
   throw std::invalid_argument("Vertical length must be 1..10000 mm; groove_index must be an integer 1..100");
  gooseneck::Options o;o.clearance=number(s,"gooseneck.clearance_mm");
  if(o.clearance<=0||o.clearance>1000)throw std::invalid_argument("Clearance must be >0 and <=1000 mm");
  o.bodyTiltDegrees=number(s,"gooseneck.body_tilt_deg");
  if(std::abs(o.bodyTiltDegrees)>30)throw std::invalid_argument("Body tilt must be -30..30 degrees");
  o.sideLeanDegrees=number(s,"gooseneck.side_lean_deg");
  if(o.sideLeanDegrees<0||o.sideLeanDegrees>20)throw std::invalid_argument("Side lean must be 0..20 degrees");
  o.positiveToolX=boolean(s,"gooseneck.tool_x_positive");o.positiveBodyZ=boolean(s,"gooseneck.tool_z_to_body");
  for(size_t i=0;i<roles.size();++i){std::array<double,6> v{};
   for(size_t j=0;j<fields.size();++j){v[j]=number(s,"gooseneck."+roles[i]+"."+fields[j]);
    if(std::abs(v[j])>(j<3?100:180))throw std::invalid_argument("Offset exceeds limit: "+roles[i]);}
   o.offsets[i]={{v[0],v[1],v[2]},{v[3],v[4],v[5]}};}
  const fs::path directory=fs::absolute(options.output_directory);fs::create_directories(directory);
  Scratch temp(directory);const auto input=temp.path/"input.ply";stageInput(options.input_ply,input,s);
  const auto geometry=extract(input.string(),height,&o);
  const auto featureStart=std::chrono::steady_clock::now();
  if(size_t(selected)>geometry.paths.size())throw std::runtime_error("Requested groove_index exceeds detected groove count "+std::to_string(geometry.paths.size()));
  const auto& path=geometry.paths.at(size_t(selected)-1);
  for(size_t i=0;i<8;++i){if(!std::isfinite(path.position[i].norm()))throw std::runtime_error("Invalid waypoint");
   if(i&&(path.position[i]-path.position[i-1]).norm()<0.5)throw std::runtime_error("Offsets create coincident adjacent waypoints");}
  writeRows(temp.path/"features.csv",path,geometry,o,int(selected));
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  if(pcl::io::loadPLYFile((temp.path/"input_colored.ply").string(),cloud)<0)throw std::runtime_error("Cannot read segmented preview");
  std::vector<gooseneck::PreviewPoint> markers;gooseneck::appendTrajectoryPreview(markers,path);
  for(const auto& marker:markers){pcl::PointXYZRGB p;p.x=marker.position.x;p.y=marker.position.y;p.z=marker.position.z;
   p.r=marker.color[0];p.g=marker.color[1];p.b=marker.color[2];cloud.push_back(p);}
  writeCloud(temp.path/"result.ply",cloud);pcl::PointCloud<pcl::PointXYZRGB> points;
  for(const auto& v:path.position){pcl::PointXYZRGB p;p.x=v.x;p.y=v.y;p.z=v.z;p.r=255;p.g=20;p.b=147;points.push_back(p);}
  writeCloud(temp.path/"feature_points.ply",points);
  std::ofstream meta(temp.path/"settings.conf");meta<<"# SDK "<<version()<<"; input="<<options.input_ply<<"; detected_grooves="<<geometry.paths.size()<<'\n';
  for(const auto& [k,v]:s)meta<<k<<'='<<v<<'\n';meta.close();if(!meta)throw std::runtime_error("Cannot save settings");
  for(const auto* suffix:{"result.ply","feature_points.ply","features.csv","settings.conf"})
   fs::rename(temp.path/suffix,directory/(options.output_prefix+"_"+suffix));
  result.csv_path=(directory/(options.output_prefix+"_features.csv")).string();
  result.visualization_ply_path=(directory/(options.output_prefix+"_result.ply")).string();
  result.feature_points_ply_path=(directory/(options.output_prefix+"_feature_points.ply")).string();
  result.seam_point_count=geometry.seamCount;result.path_point_count=8;
  result.feature_time_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-featureStart).count();
  result.status=0;result.message="ok";
  std::cout<<"Gooseneck SDK "<<version()<<": selected groove "<<int(selected)<<'/'<<geometry.paths.size()
   <<", 8 points, tool +X="<<(o.positiveToolX?"+transverse":"-transverse")
   <<", body tilt="<<o.bodyTiltDegrees<<" deg, outward side lean="<<o.sideLeanDegrees
   <<" deg (no collision guarantee)"<<'\n';
 }catch(const std::exception& e){result.message=e.what();}
 result.total_time_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 result.primary_time_ms=result.total_time_ms-result.feature_time_ms;return result;
}
