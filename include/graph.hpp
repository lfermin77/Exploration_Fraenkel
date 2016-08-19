







class SLAM_node{
	public:
	float distance_from_origin;
	std::complex<double> position;
	vector<int> edges;
	
	SLAM_node(){
		distance_from_origin=-1;
	}

};

class SLAM_edge{
	public:
	float distance;
	SLAM_node* from;
	SLAM_node* to;

	SLAM_edge(){
		distance=-1;
	}
		
};
	
class UtilityGraph{
	std::vector<SLAM_node> Nodes;
	std::vector<SLAM_edge> Edges;
	
	public:
	
	int find_point_in_node(geometry_msgs::Point node_position){
		int index=-1;
		std::complex<double> query_position(node_position.x, node_position.y);

		for (int i=0; i < Nodes.size();i++){
			float distance = std::norm( Nodes[i].position - query_position) ;
			if(distance==0){
				index=i;
			}
		}
		
		return index;
	}

	void add_edge(geometry_msgs::Point node_position_from , geometry_msgs::Point node_position){
		int a=1;
	}
	
	
	void build_graph_from_edges(std::vector<geometry_msgs::Point> edge_markers){
		int number_of_edges = edge_markers.size()/2;
		
		Edges.resize(number_of_edges);
		
		for (int i=0; i < number_of_edges;i++){

			
			int index_from = find_point_in_node ( edge_markers[2*i] );
			if( index_from < 0){
				std::complex<double> node_position(edge_markers[2*i].x, edge_markers[2*i].y);

				SLAM_node node_from;
				node_from.position = node_position;
				node_from.edges.push_back(i);		
				Nodes.push_back(node_from);		
				index_from = Nodes.size()-1;//the last term 
			}
			
			int index_to   = find_point_in_node ( edge_markers[i+1] );
			if( index_to < 0){
				std::complex<double> node_position(edge_markers[2*i+1].x, edge_markers[2*i+1].y);

				SLAM_node node_to;
				node_to.position = node_position;
				node_to.edges.push_back(i);				
				Nodes.push_back(node_to);		
				index_to = Nodes.size()-1;//the last term
			}
			
			
		}		
		
		
		
	}
	
};
