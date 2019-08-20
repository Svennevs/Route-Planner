#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {  	
  	int i=0;
	for (Model::Node node : this->Nodes() ){
    	m_Nodes.push_back(Node(i, this, node));
        i++;
	}
  	CreateNodeToRoadHashmap();
}

RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices){
	//loop through node_indices to find closest unvisited
  	Node *closest_node = nullptr;
  	Node node;
  	
  	for(int node_idx : node_indices){
      	node = parent_model->SNodes()[node_idx];
      	if(this->distance(node) !=0 && !node.visited){
          if(closest_node == nullptr || this->distance(node) < this->distance(*closest_node)){
            closest_node = &parent_model->SNodes()[node_idx];
          }
        }
    }
  	return closest_node;
}

void RouteModel::Node::FindNeighbors(){
  //search for each road of a node for the nearest neighbor and push it to this->neighbors
  for(auto & road : parent_model->node_to_road[this->index]){
    RouteModel::Node *closest = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if(closest){
    	this->neighbors.emplace_back(closest);   
    }
  }
}

RouteModel::Node & RouteModel::FindClosestNode(float x, float y){
  Node node;
  node.x = x; 
  node.y = y;
  
  float min_dist = std::numeric_limits<float>::max();
  float dist;
  int closest_idx;

  for(const Model::Road & road : Roads()){ //go through all roads
    if(road.type != Model::Road::Type::Footway){ //if not footway
      for(int node_idx : Ways()[road.way].nodes ){ //go through all nodes of this road
        dist = node.distance(SNodes()[node_idx]);
        if(dist<min_dist){
          closest_idx=node_idx;
          min_dist = dist;
        }
      }
    }
  }
  return SNodes()[closest_idx];
}

void RouteModel::CreateNodeToRoadHashmap(){
	for (const Model::Road &road : Roads()){  //for each reference
      if(road.type != Model::Road::Type::Footway){
      	for(int node_idx : Ways()[road.way].nodes ){
          if (node_to_road.find(node_idx) == node_to_road.end()){
          	node_to_road[node_idx] = std::vector< const Model::Road *> ();
          }
          node_to_road[node_idx].push_back(&road); //pushes the pointer
        }
      }
    }

}
