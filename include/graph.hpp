


class SLAM_node{
	public:
	float distance_from_origin;
	std::complex<double> position;
	vector<int> edges;

};

class SLAM_edge{
	public:
	float distance;
	SLAM_node* from;
	SLAM_node* to;
	void * pepe;
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
		
		for (int i=0; i < edge_markers.size();i+=2){

			SLAM_edge current_edge;
			Edges.push_back(current_edge);
			
			Edges.back().distance = -1;
			
			int index_from = find_point_in_node ( edge_markers[i] );
			if( index_from<0){
				SLAM_node node_from;
				node_from.distance_from_origin = -1;
//				node_from.position
				
			}
			
			int index_to   = find_point_in_node ( edge_markers[i+1] );
			
//			asdf
		}		
		
		
		
	}
	
};
