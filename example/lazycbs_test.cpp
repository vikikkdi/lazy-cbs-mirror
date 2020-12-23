#include <yaml-cpp/yaml.h>

#include <lazycbs/map_loader.h>
#include <lazycbs/agents_loader.h>
#include <lazycbs/egraph_reader.h>
#include <lazycbs/mapf-solver.h>

#include <string>
#include <cstring>
#include <fstream>
#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <utility>
#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

#include <boost/functional/hash.hpp>

#include <chrono>

using namespace std;


#ifndef EXTRA_PEN
#define EXTRA_PEN 0  /* Off by default. */
#endif

/* This flag controls early solver termination */
volatile static sig_atomic_t terminated = 0;

/* The signal handler just sets the flag. */
void catch_int (int sig) {
  terminated = 1;
  // signal (sig, catch_int);
}
void set_handlers(void) {
  signal(SIGINT, catch_int);
}
void clear_handlers(void) {
  signal(SIGINT, SIG_DFL);
}


int main(int argc, char* argv[]) {

  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");

  std::string inputFile;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  std::vector<std::pair<int, int> > obstacles;
  std::vector<std::pair<int, int> > goals;
  std::vector<std::pair<int, int> > starts;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.emplace_back(std::make_pair(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    starts.emplace_back(std::make_pair(start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(std::make_pair(goal[0].as<int>(), goal[1].as<int>()));
  }

  std::pair<int, std::vector< std::vector< std::pair<int, int> > > > solution;

  set_handlers();
  // read the map file and construct its two-dim array

  lazycbs::MapLoader ml = lazycbs::MapLoader(dimx, dimy, obstacles);
  
  // read agents' start and goal locations
  lazycbs::AgentsLoader al = lazycbs::AgentsLoader(starts, goals);

  // read the egraph (egraph file, experience_weight, weigthedastar_weight)
  lazycbs::EgraphReader egr;
lazycbs::MAPF_Solver mapf1(ml, al, egr, 1e8);

  //ofstream res_f;
  //res_f.open(results_fname, ios::app);  // append the results file

  //bool res;
  //cout << rrr_it << " ; ";

  clear_handlers();
  // ECBSSearch ecbs = ECBSSearch(ml, al, egr, w_hwy, w_focal, tweakGVal, rrr_it, rand_succ_gen);
  // ecbs.time_limit_cutoff = time_limit;
  // bool res = ecbs.runECBSSearch();
  // ecbs.printPaths();

  auto lazycbs_start = std::chrono::system_clock::now();
  bool success = lazycbs::MAPF_MinCost(mapf1);
  if(success)
    mapf1.getPaths(&solution);
  std::cout<<"Cost :: "<<solution.first<<std::endl;
  auto lazycbs_end = std::chrono::system_clock::now();
  auto lazycbs_time = std::chrono::duration<double>(lazycbs_end - lazycbs_start).count();
  //mapf_icts.search(mapf, starts, &solution1);//
  if (success) {
    std::cout << "Planning successful! " << std::endl;

    std::ofstream out("../example/output_lazycbs.yaml");
    out << "statistics:" << std::endl;
    out << "  cost: " << solution.first << std::endl;
    out << "  runtime: " << lazycbs_time << std::endl;
    out << "schedule:" << std::endl;

    int count = 0;

    for(auto it=solution.second.begin(); it!=solution.second.end();++it){
      out << "  agent" << count << ":" << std::endl;
      std::vector<std::pair<int, int> > output;
      output = *it;
      for(int i=0; i<output.size(); i++){
        out << "    - x: " << output[i].first << std::endl
          << "      y: " << output[i].second << std::endl
          << "      t: " << i << std::endl;
      }
      count++;
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  std::cout<<"TIME TAKEN TO COMPLETE THE TASK ::"<<std::endl
            <<"LAZYCBS :: "<<lazycbs_time<<std::endl<<std::endl<<std::endl;


  return 0;
}