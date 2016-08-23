#include <graph_filter/Constraints.h>


Constraints::Constraints(std::map<int, int> & req_nums_):req_nums(req_nums_) {
	// reset();
}

Constraints::~Constraints() {
}



bool Constraints::passLabel(int sid, int l) {
  std::cout << "Not supposed to call passlabel in class Constraints!" << std::endl;
  return false;
}

std::vector<int> Constraints::getVerticeList() {
  std::vector<int> result;
  for(std::map<int, int>::iterator it = req_nums.begin(); it != req_nums.end(); it++) {
     result.push_back(it->first);
  }
  return result;
}


std::string Constraints::getClassName() {
	std::cout << "[class] Constraints " << std::endl;
	return "Constraints";
}
