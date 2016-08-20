#include "graph.hpp"



////Graph as adjacency list nested class

node_info::node_info(){
	distance_from_origin= std::numeric_limits<float>::infinity();
	label=-1;
}
/////////////////////////////

edge_info::edge_info(){
	distance = -1;
}


///////////////////
	
Node::Node(){
	predecesor=NULL;
}


void Node::print_node_label_and_pos(){
	std::cout << "Node "<< info.label<< std::endl;
	for(int i=0; i < connected.size();i++){
		std::cout << "connected to "<< (connected[i].to)->info.label<<std::endl;
		std::cout << "connected in ("<< (connected[i].linker)->from->info.label<<","<< (connected[i].linker)->to->info.label << ")"<<std::endl;
	}
}

/////////////////////	

UtilityGraph::~UtilityGraph(){
	for(Node_iter it = Nodes.begin(); it != Nodes.end(); it++)
	    delete *it; 
	Nodes.clear();

	for(Edge_iter it = Edges.begin(); it != Edges.end(); it++)
	    delete *it;
	Edges.clear();

}



void UtilityGraph::print_nodes(){
	for(Node_iter it = Nodes.begin(); it != Nodes.end(); it++){
		(*it)->print_node_label_and_pos();
//			cout << "all labels "<<(*it)->info.label<<endl;
	}
}


Node_iter UtilityGraph::find_point_in_node(std::complex<double> query_position){
	Node_iter first = Nodes.begin();
	Node_iter last = Nodes.end();

	while (first!=last) {
		if ((*first)->info.position == query_position) return first;
		++first;
	}
	return last;
}

void UtilityGraph::build_graph_from_edges(std::vector<geometry_msgs::Point> edge_markers){
	int number_of_edges = edge_markers.size()/2;				
	int labeler=0;
	
	Node* from_Node_ptr; 
	Node* to_Node_ptr;   
			
	for (int i=0; i < number_of_edges;i++){
	
//		cout << "Insert nodes"<< endl;
		// FROM
		std::complex<double> FROM_position(edge_markers[2*i].x, edge_markers[2*i].y);			
		Node_iter FROM_node_iter = find_point_in_node(FROM_position);		
		if(FROM_node_iter == Nodes.end() ){//couldn't find, insert new node
			labeler++;
			from_Node_ptr =new Node;
			from_Node_ptr->info.position = FROM_position;
			from_Node_ptr->info.label = labeler;
			Nodes.push_back(from_Node_ptr);	
		}			
		else from_Node_ptr = *FROM_node_iter;
		
		// TO
		std::complex<double> TO_position(edge_markers[2*i+1].x, edge_markers[2*i+1].y);			
		Node_iter TO_node_iter = find_point_in_node(TO_position);			
		if(TO_node_iter == Nodes.end() ){//couldn't find, insert new node
			labeler++;
			to_Node_ptr = new Node;
			to_Node_ptr->info.position = TO_position;
			to_Node_ptr->info.label = labeler;
			Nodes.push_back(to_Node_ptr);				
		}			
		else to_Node_ptr = *TO_node_iter;


//		cout << "Insert Edges"<<endl;
		Edge* current_edge = new Edge;
		current_edge->info.distance = abs(FROM_position - TO_position);
		
		current_edge->from = from_Node_ptr;
		current_edge->to   = to_Node_ptr;  
		
		Edges.push_back(current_edge);
	
//		cout << "Insert Linked Information"<<endl;
		connections connecting_from;
		connecting_from.linker = current_edge;			
		connecting_from.to = to_Node_ptr;
		from_Node_ptr->connected.push_back(connecting_from);
		
		connections connecting_to;
		connecting_to.linker = current_edge;			
		connecting_to.to = from_Node_ptr;
		to_Node_ptr->connected.push_back(connecting_to);
		//Edges
		current_edge->from = from_Node_ptr;
		Edges.back()->to = to_Node_ptr;
	}
	std::cout << "Found " << labeler  <<" nodes"<< std::endl;
	
}





