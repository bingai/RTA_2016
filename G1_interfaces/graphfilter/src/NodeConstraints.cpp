#include <graph_filter/NodeConstraints.h>


NodeConstraints::NodeConstraints(std::map<int, int> & req_nums_):
  Constraints(req_nums_) {
	reset();
}

NodeConstraints::~NodeConstraints() {
}


void NodeConstraints::reset() {
	success_nums.clear();	
}

int NodeConstraints::check(int a) {
  std::cout << "Error! You are not supposed to call this unary constraint" << std::endl; 
  return 0;
}


bool NodeConstraints::passLabel(int sid, int l) {
  if (req_nums.find(l) == req_nums.end()) {
    std::cout << "Label: " << l << " is not in the rejection list!" << std::endl;
  	return false;
  }
  if (success_nums.find(sid) != success_nums.end()) {
    return success_nums[sid] >= req_nums[l];
  }

  int nums = check(sid);
  success_nums.insert(std::pair<int, int>(sid, nums) );
  return nums >= req_nums[l];
}

std::string NodeConstraints::getClassName() {
  std::cout << "[class] NodeConstraints " << std::endl;
  return "NodeConstraints";
}
