#ifndef NODECONSTRAINTS_H
#define NODECONSTRAINTS_H

#include <stdio.h>
#include <map>
#include <iostream>
#include <vector>
#include <graph_filter/Constraints.h>

class NodeConstraints: public Constraints {
  
  protected:
    //checked scene graph number to success numbers
    std::map<int, int> success_nums;
  public:

    NodeConstraints(std::map<int, int> & req_nums_);
    ~NodeConstraints();

    void reset();
    virtual int check(int a);
    // virtual bool check(int a, int b);
    virtual bool passLabel(int sid, int l);
    virtual std::string getClassName();

};

#endif