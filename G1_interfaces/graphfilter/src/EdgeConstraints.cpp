#include <graph_filter/EdgeConstraints.h>

EdgeConstraints::EdgeConstraints(std::vector<std::set<int> > & nbgh_list_,
  std::map <int, std::set<int> > & scene2sub_, bool directed_,
  std::map<int, std::set<int> > subDest_,
  std::map<int, int> & req_nums_)
  :Constraints(req_nums_),
  nbgh_list(nbgh_list_), scene2sub(scene2sub_), 
  directed(directed_), subDest(subDest_){
  std::cout << "In counstructor " << subDest_.size() << std::endl;
  std::cout << "counstructor " << subDest.size() << std::endl;

	reset();
}

EdgeConstraints::~EdgeConstraints() {
}


void EdgeConstraints::reset() {

}

int EdgeConstraints::check(int a, int b) {
  std::cout << "Error! You are not supposed to call this unary constraint" << std::endl; 
  return 0;
}

bool EdgeConstraints::setOverlap(const std::set<int>& A, const std::set<int>& B) {
  for (std::set<int>::iterator it = A.begin(); it != A.end(); ++it) {
    if (B.find(*it) != B.end() ) {
      return true;
    }
  }
  return false; 
}

bool EdgeConstraints::isValid(int id, int l) {
  // std::cout << "in isValid " << l << std::endl;
  // std::cout << "A size" << scene2sub[id].size() << std::endl;
  // std::cout << "subdest size" << subDest.size() << std::endl;
  // std::cout << "B size" << subDest[l].size() << std::endl;  
  if (setOverlap(scene2sub[id], subDest[l]) ) {
    return true;
  } else {
    return false;
  }
}


bool EdgeConstraints::edgeCheckedBefore(int sid, int nid) {
  int hashKey = sid * hashBase + nid;
  if (edgesResult.find(hashKey) != edgesResult.end()) {
    return true;
  } else {
    return false;
  }

}

bool EdgeConstraints::putEdgeCheckResult(int sid, int nid, int result) {
  int hashKey = sid * hashBase + nid;
  edgesResult.insert(std::pair<int, int>(hashKey, result) );
  if (!directed) {
    int hashReverse = nid * hashBase + sid;
    edgesResult.insert(std::pair<int, int>(hashReverse, result) );
  }
  // check edge relationship in the nbgh
  // if result is positive 
  if (result > 0) { 
      nbgh_list[sid].insert(nid);
  } else {
    std::set<int>::iterator it = nbgh_list[sid].find(nid);
    if (it != nbgh_list[sid].end() ) {
      nbgh_list[sid].erase(it);
    }
  }

  if (!directed) {
    if (result > 0) { 
        nbgh_list[nid].insert(sid);
    } else {
      std::set<int>::iterator it = nbgh_list[nid].find(sid);
      if (it != nbgh_list[nid].end() ) {
        nbgh_list[nid].erase(it);
      }
    }

  }

}

int EdgeConstraints::getEdgeCheckResult(int sid, int nid) {
  if (!edgeCheckedBefore(sid, nid)) {
    int result = check(sid, nid);
    putEdgeCheckResult(sid, nid, result);
  }
  int hashKey = sid * hashBase + nid;
  return edgesResult[hashKey];
}


std::vector<int> EdgeConstraints::getNeighbors(int id) {
  std::cout << "You are not supposed to call getNeighbors in EdgeConstraints!" << std::endl;
  return std::vector<int>();
}

bool EdgeConstraints::passLabel(int sid, int l) {
  std::vector<int> neighbors = getNeighbors(sid);
  int count = 0;
  // std::cout << "after getNeighbors" << std::endl;
  // std::cout << "subdest size pass" << subDest.size() << std::endl;

  for (std::vector<int>::iterator it = neighbors.begin(); it != neighbors.end(); ++it) {
    if (!isValid(*it, l)) {
       // std::cout << "after isValid " << std::endl;

      continue;
    }
    count += getEdgeCheckResult(sid, *it);
  } 

  // std::cout << "after for count" << std::endl;

  return count >= req_nums[l];
}

std::string EdgeConstraints::getClassName() {
  std::cout << "[class] EdgeConstraints " << std::endl;
  return "EdgeConstraints";
}
