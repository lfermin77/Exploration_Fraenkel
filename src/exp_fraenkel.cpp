//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>

//openCV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

//DuDe
#include "inc_decomp.hpp"




class ROS_handler
{
	ros::NodeHandle n;
	
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;	
	cv_bridge::CvImagePtr cv_ptr;
		
	std::string mapname_;
	ros::Subscriber map_sub_;	
	ros::Timer timer;
			
	float Decomp_threshold_;
	Incremental_Decomposer inc_decomp;
	Stable_graph Stable;

	std::vector <double> clean_time_vector, decomp_time_vector, paint_time_vector, complete_time_vector;

	ros::Subscriber odom_sub_;
	
	public:
		ROS_handler(const std::string& mapname, float threshold) : mapname_(mapname),  it_(n), Decomp_threshold_(threshold)
		{
			ROS_INFO("Waiting for the map");
			map_sub_ = n.subscribe("map", 2, &ROS_handler::mapCallback, this); //mapname_ to include different name
			odom_sub_ = n.subscribe("pose_corrected", 1, &ROS_handler::odomCallback, this);
			
			timer = n.createTimer(ros::Duration(0.5), &ROS_handler::metronomeCallback, this);
			image_pub_ = it_.advertise("/image_frontier", 1);
			
			
			cv_ptr.reset (new cv_bridge::CvImage);
			cv_ptr->encoding = "mono8";
		}

		~ROS_handler()	{
		}


/////////////////////////////	
// ROS CALLBACKS			
////////////////////////////////		

		void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
		{
			double begin_process, end_process, begin_whole, occupancy_time, decompose_time, drawPublish_time, whole_time;
			begin_whole = begin_process = getTime();
			
			ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);

		///////////////////////Occupancy to clean image	
			cv::Mat grad, img(map->info.height, map->info.width, CV_8U);
			img.data = (unsigned char *)(&(map->data[0]) );
			
			float pixel_Tau = Decomp_threshold_ / map->info.resolution;				
			cv_ptr->header = map->header;
			cv::Point2f origin = cv::Point2f(map->info.origin.position.x, map->info.origin.position.y);
			cv::Rect first_rect = find_image_bounding_Rect(img); 

			cv::Mat cropped_img;
			img(first_rect).copyTo(cropped_img);

			cv::Mat black_image, image_cleaned = clean_image2(img, black_image);
			
			end_process = getTime();	occupancy_time = end_process - begin_process;




			begin_process = getTime();


			
			end_process = getTime();	
			cout << "time for contour " <<  end_process - begin_process << endl;
			


		///////////////////////// Decompose Image
			begin_process = getTime();
			
			Stable = inc_decomp.decompose_image(image_cleaned, pixel_Tau, origin, map->info.resolution);
			
			end_process = getTime();	decompose_time = end_process - begin_process;
			
		////////////Draw Image & publish
			begin_process = getTime();

	//		cv::Mat croppedRef(Colored_Frontier, resize_rect);			
			cv::flip(black_image, black_image,0);  cv::Mat big = Stable.draw_stable_contour() & ~black_image;

			cout << "Rect "<< first_rect << endl;

//			cv::Mat croppedRef(big, first_rect); cv::Mat croppedImage; croppedRef.copyTo(croppedImage);			grad = croppedImage;
			cv::Mat cropped;
			big(first_rect).copyTo(cropped);
			
			
			cv::Size image_size = image_cleaned.size(); 
			grad =  cv::Mat::zeros(image_size.height, image_size.width, CV_8UC1);
			cropped.copyTo(grad(first_rect));

			image_cleaned = clean_image2(img, black_image);


			grad= image_cleaned;


			cv_ptr->encoding = "32FC1";
			grad.convertTo(grad, CV_32F);
			grad.copyTo(cv_ptr->image);////most important
			
			end_process = getTime();	drawPublish_time = end_process - begin_process;
			
			whole_time = end_process - begin_whole;

			printf("Time: total %.0f, Classified: occ %.1f, Decomp %.1f, Draw %.1f \n", whole_time, occupancy_time, decompose_time, drawPublish_time);


			clean_time_vector.push_back(occupancy_time);
			decomp_time_vector.push_back(decompose_time);
			paint_time_vector.push_back(drawPublish_time);
			complete_time_vector.push_back(whole_time);
			
			cout << "Time Vector size "<< clean_time_vector.size() << endl;
			for(int i=0; i < clean_time_vector.size(); i++){
//				cout << time_vector[i] << endl;
				printf("%.0f %.0f %.0f %.0f \n", paint_time_vector[i], clean_time_vector[i],  decomp_time_vector[i] , complete_time_vector[i]);
			}
		/////////////////////////	
		}
			

/////////////////		
		void metronomeCallback(const ros::TimerEvent&)
		{
//		  ROS_INFO("tic tac");
		  publish_Image();
		}


////////////////
		void odomCallback(const nav_msgs::Odometry& msg)
		{
			float x =  msg.pose.pose.position.x;
			float y =  msg.pose.pose.position.y;
			
			float yaw = tf::getYaw(msg.pose.pose.orientation);
			



		}




////////////////////////
// PUBLISHING METHODS		
////////////////////////////		
		void publish_Image(){
			image_pub_.publish(cv_ptr->toImageMsg());
		}



/////////////////////////
//// UTILITY
/////////////////////////

		cv::Mat clean_image(cv::Mat Occ_Image, cv::Mat &black_image){
			//Occupancy Image to Free Space	
			
			cv::Mat valid_image = Occ_Image < 101;
			std::vector<std::vector<cv::Point> > test_contour;
			cv::findContours(valid_image, test_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

			cv::Rect first_rect = cv::boundingRect(test_contour[0]);
			for(int i=1; i < test_contour.size(); i++){
				first_rect |= cv::boundingRect(test_contour[i]);
			}
			cv::Mat reduced_Image;
			valid_image(first_rect).copyTo(reduced_Image);
			
			
			cv::Mat open_space = reduced_Image<10;
			black_image = reduced_Image>90 & reduced_Image<=100;		
			cv::Mat Median_Image, out_image, temp_image ;
			int filter_size=2;

			cv::boxFilter(black_image, temp_image, -1, cv::Size(filter_size, filter_size), cv::Point(-1,-1), false, cv::BORDER_DEFAULT ); // filter open_space
			black_image = temp_image > filter_size*filter_size/2;  // threshold in filtered
			cv::dilate(black_image, black_image, cv::Mat(), cv::Point(-1,-1), 4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			// inflate obstacle

			filter_size=10;
			cv::boxFilter(open_space, temp_image, -1, cv::Size(filter_size, filter_size), cv::Point(-1,-1), false, cv::BORDER_DEFAULT ); // filter open_space
			Median_Image = temp_image > filter_size*filter_size/2;  // threshold in filtered
			Median_Image = Median_Image | open_space ;
			//cv::medianBlur(Median_Image, Median_Image, 3);
			cv::dilate(Median_Image, Median_Image,cv::Mat());

			out_image = Median_Image & ~black_image;// Open space without obstacles




			cv::Size image_size = Occ_Image.size();
			cv::Mat image_out(image_size, CV_8UC1);
			cv::Mat black_image_out(image_size, CV_8UC1) ; 

			out_image.copyTo(image_out(first_rect));
			
			black_image.copyTo(black_image_out(first_rect));
			black_image =black_image_out;

			return image_out;
		}


		cv::Mat clean_image2(cv::Mat Occ_Image, cv::Mat &black_image){
			//Occupancy Image to Free Space	
			cv::Mat open_space = Occ_Image<10;
			black_image = Occ_Image>90 & Occ_Image<=100;		
			cv::Mat Median_Image, out_image, temp_image ;
			int filter_size=2;

			cv::boxFilter(black_image, temp_image, -1, cv::Size(filter_size, filter_size), cv::Point(-1,-1), false, cv::BORDER_DEFAULT ); // filter open_space
			black_image = temp_image > filter_size*filter_size/2;  // threshold in filtered
			cv::dilate(black_image, black_image, cv::Mat(), cv::Point(-1,-1), 4, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );			// inflate obstacle

			filter_size=10;
			cv::boxFilter(open_space, temp_image, -1, cv::Size(filter_size, filter_size), cv::Point(-1,-1), false, cv::BORDER_DEFAULT ); // filter open_space
			Median_Image = temp_image > filter_size*filter_size/2;  // threshold in filtered
			Median_Image = Median_Image | open_space ;
			//cv::medianBlur(Median_Image, Median_Image, 3);
			cv::dilate(Median_Image, Median_Image,cv::Mat());

			out_image = Median_Image & ~black_image;// Open space without obstacles

			return out_image;
		}

		cv::Rect find_image_bounding_Rect(cv::Mat Occ_Image){
			cv::Mat valid_image = Occ_Image < 101;
			std::vector<std::vector<cv::Point> > test_contour;
			cv::findContours(valid_image, test_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );

			cv::Rect first_rect = cv::boundingRect(test_contour[0]);
			for(int i=1; i < test_contour.size(); i++){
				first_rect |= cv::boundingRect(test_contour[i]);
			}
			return first_rect;
		}
			

};










int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Inc_Dual_Decomposer");
	
	std::string mapname = "map";
	
	float decomp_th=3;
	if (argc ==2){ decomp_th = atof(argv[1]); }	
	
	ROS_handler mg(mapname, decomp_th);
	ros::spin();
	
	return 0;
}
