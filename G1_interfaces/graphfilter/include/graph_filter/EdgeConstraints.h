#ifndef EDGECONSTRAINTS_H
#define EDGECONSTRAINTS_H

#include <stdio.h>
#include <map>
#include <iostream>
#include <vector>
#include <set>
#include <graph_filter/Constraints.h>

class EdgeConstraints: public Constraints {
  
  private:

  protected:
    static const int hashBase = 3000;
    std::vector<std::set<int> > & nbgh_list;
    std::map <int, std::set<int> > & scene2sub;
    // key represents an edge, value can only be 1 or 0
    std::map<int, int> edgesResult;
    //key is the subgraph label in the req_num, value is the set of destination label expected 
    std::map<int, std::set<int> > subDest;
    bool directed;
    
    bool setOverlap(const std::set<int>& A, const std::set<int>& B);
    bool edgeCheckedBefore(int sid, int nid);
    bool putEdgeCheckResult(int sid, int nid, int result);
    int getEdgeCheckResult(int sid, int nid);
    bool isValid(int id, int l);

  public:

    EdgeConstraints(std::vector<std::set<int> > & nbgh_list,
      std::map <int, std::set<int> > & scene2sub, bool directed,
      std::map<int, std::set<int> > subDest_,
      std::map<int, int> & req_nums_);
    ~EdgeConstraints();

    void reset();
    virtual int check(int a, int b);
    virtual bool passLabel(int sid, int l);
    virtual std::string getClassName();
    virtual std::vector<int> getNeighbors(int id);
    
};

#endif