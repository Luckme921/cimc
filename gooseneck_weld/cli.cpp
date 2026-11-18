#include <gooseneck_sdk/gooseneck_sdk.hpp>
#include <iostream>
int main(int argc,char** argv) {
 if(argc==2&&std::string(argv[1])=="--version"){std::cout<<gooseneck_sdk::version()<<'\n';return 0;}
 if(argc<4){std::cerr<<"Usage: gooseneck_weld_extractor input.ply output_directory prefix [--config file] [--set key=value]...\n";return 2;}
 gooseneck_sdk::RunOptions options;
 options.input_ply=argv[1];options.output_directory=argv[2];options.output_prefix=argv[3];
 for(int i=4;i<argc;++i){const std::string flag=argv[i];
  if(i+1>=argc){std::cerr<<"Missing option value\n";return 2;}
  if(flag=="--set")options.parameter_overrides.push_back(argv[++i]);
  else if(flag=="--config")options.config_file=argv[++i];
  else {std::cerr<<"Unknown option: "<<flag<<'\n';return 2;}}
 const auto result=gooseneck_sdk::run(options);
 if(result.status){std::cerr<<result.message<<'\n';return 1;}
 std::cout<<"CSV: "<<result.csv_path<<"\nPLY: "<<result.visualization_ply_path<<"\nTotal: "<<result.total_time_ms<<" ms\n";
}
