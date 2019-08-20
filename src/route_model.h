#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
      	Node * parent = nullptr;
      	float h_value = std::numeric_limits<float>::max();
      	float g_value = 0.0;
      	bool visited = false;
      	std::vector<Node*> neighbors;
       	float distance(Node othernode) const {
        	return std::sqrt(std::pow( (x - othernode.x),2) +  std::pow((y - othernode.y),2));
        } 
        void FindNeighbors();
     	
      	
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model) {index = idx;}
      
      private:
        // Add private Node variables and methods here.
      	int index;	
        RouteModel * parent_model = nullptr;
      	Node * FindNeighbor(std::vector<int> node_indices);
    };
    
    // Add public RouteModel variables and methods here.
  	auto &GetNodeToRoadMap(){return node_to_road;} //returns reference to private var node_to_road
    std::vector<Node> &SNodes() {return m_Nodes;} //returns reference to private var m_Nodes
    RouteModel(const std::vector<std::byte> &xml);  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.
  	Node & FindClosestNode(float x, float y);
  
  private:
  	std::unordered_map<int, std::vector<const Model::Road *>> node_to_road; //unordered map with int keys and vector of const model::Road pointers
  	void CreateNodeToRoadHashmap();
  	std::vector<Node> m_Nodes;
  	
 

};

#endif