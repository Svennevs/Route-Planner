#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {  //m_model returns a reference to model, so has to be in the initializer list
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y); 
}  

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
  distance  = 0.0f;
  std::vector<RouteModel::Node>  path_found;
  RouteModel::Node parentnode;
  
  while(current_node->parent != nullptr){
    path_found.push_back(*current_node);
    parentnode = *(current_node->parent);
    distance += current_node->distance(parentnode);
    current_node = current_node->parent;
  }
  path_found.push_back(*current_node);  
  distance = distance*m_Model.MetricScale();
  return path_found;
}

void RoutePlanner::AStarSearch(){
  start_node->visited = true;
  open_list.push_back(start_node);
  RouteModel::Node *current_node = nullptr;
  while(open_list.size() > 0){
    current_node = NextNode();
    if(current_node->distance(* end_node) == 0 ){
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }
    AddNeighbors(current_node);
  }
}

float RoutePlanner::CalculateHValue(RouteModel::Node * current_node_ptr){
  return current_node_ptr->distance(*end_node);
}


RouteModel::Node * RoutePlanner::NextNode(){
  	std::sort( open_list.begin(),open_list.end(), [](const auto &v1, const auto &v2) {
    	return v1->g_value+v1->h_value < v2->g_value + v2->h_value;
      	});
  	RouteModel::Node *lowestcopy = open_list.front(); //.front returns  a pointer
  	open_list.erase(open_list.begin()); //.begin returns an iterator
  	return lowestcopy;

}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node_ptr){
  current_node_ptr->FindNeighbors(); //populate current nodes neighbors vector
  for(RouteModel::Node *neighbor : current_node_ptr->neighbors ){
    neighbor->parent = current_node_ptr;
    neighbor->g_value = current_node_ptr->g_value + current_node_ptr->distance(*neighbor);
    neighbor->h_value = CalculateHValue(neighbor);
    open_list.push_back(neighbor);
    neighbor->visited = true;
  }
  
}








