#include "astar_planner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_planner{
    AstarPlanner::AstarPlanner(){}
    // Don't need this constructor here. Movebase will call initialize.
    /*AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }*/

    void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            divide=6;
            range=M_PI/3;
            chordl=0.13;
            phase = range/divide;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            resolution=costmap_->getResolution();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            OGM.resize(map_size);

            //vector<unsigned int> coo;

            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    unsigned int cost = costmap_->getCost(i, j);
                    /*if(cost>0){
                        coo.push_back(cost);
                    }*/
                    //get_cost << cost << endl;
                    //cout << "i:, j:" << cost << endl;

                    if (cost <= 128)
                        OGM[j * width + i] = true;
                    else{
                        //ROS_INFO("%d is false",j * width + i);
                        OGM[j * width + i] = false;
                    }
                }
            }
            /*unsigned int avr;
            for (int i = 0; i < coo.size(); ++i) {
                avr = avr + coo[i];
            }
            avr = avr / coo.size();
            ROS_INFO("%d",avr);*/

            frame_id_ = costmap_ros->getGlobalFrameID();

            ros::NodeHandle private_nh("~/" + name);

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }
    //start is current pose. goal is provided by user, both of which are global pose. plan is where plan is stored.
    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
            if (!initialized_) {
                ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
                return false;
            }
            //world coordinate to map coordinate to index
            ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                     goal.pose.position.x, goal.pose.position.y);
            double wx = start.pose.position.x;
            double wy = start.pose.position.y;
            unsigned int start_x, start_y;
            costmap_->worldToMap(wx, wy, start_x, start_y);
            int start_index = costmap_->getIndex(start_x, start_y);


            wx = goal.pose.position.x;
            wy = goal.pose.position.y;

            unsigned int goal_x, goal_y;
            costmap_->worldToMap(wx, wy, goal_x, goal_y);
            int goal_index = costmap_->getIndex(goal_x, goal_y);

            //meijibayong
            //create 2 vectors the sequence is index.
            vector<float> gCost(map_size, infinity);
            vector<int> cameFrom(map_size, -1);
            //vector <vector<geometry_msgs::PoseStamped>> trajectory(map_size);

            //openlist
            multiset <Node> openlist;

            gCost[start_index] = 0;
            //Initialize currentNode and put it in openlist
            Node currentNode;
            currentNode.index = start_index;
            currentNode.cost = gCost[start_index] + 0 + costmap_->getCost(start_x, start_y);
            openlist.insert(currentNode);

            plan.clear();

            while (!openlist.empty()) {
                // Take the element from the top(begin is the least cost)
                currentNode = *openlist.begin();
                //Delete the element from the top
                openlist.erase(openlist.begin());
                if (currentNode.index == goal_index) {
                    break;
                }
                // Get neighbors
                vector<int> neighborIndexes = get_neighbors(currentNode.index);

                for (int i = 0; i < static_cast<int>(neighborIndexes.size()); i++) {
                    //if not expanded
                    if (cameFrom[neighborIndexes[i]] == -1) {
                        //predecessor cost+ movecost
                        unsigned int nx, ny;
                        costmap_->indexToCells(neighborIndexes[i], nx, ny);
                        gCost[neighborIndexes[i]] =
                                gCost[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]) +
                                costmap_->getCost(nx, ny);
                        Node nextNode;
                        nextNode.index = neighborIndexes[i];
                        //nextNode.cost = gCost[neighborIndexes[i]];    //Dijkstra Algorithm
                        //manhattan distance
                        nextNode.cost = gCost[neighborIndexes[i]] +
                                        getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                        cameFrom[neighborIndexes[i]] = currentNode.index;
                        //put expanded node into openlist
                        openlist.insert(nextNode);
                    }
                }
            }
            //if goal does not come from any node
            if (cameFrom[goal_index] == -1) {
                cout << "Goal not reachable, failed making a global path." << endl;
                return false;
            }

            if (start_index == goal_index)
                return false;
            //Finding the best path, take note of the index
            vector<int> bestPath;
            currentNode.index = goal_index;
            while (currentNode.index != start_index) {
                bestPath.push_back(cameFrom[currentNode.index]);
                currentNode.index = cameFrom[currentNode.index];
            }
            reverse(bestPath.begin(), bestPath.end());

            ros::Time plan_time = ros::Time::now();
            for (int i = 0; i < static_cast<int>(bestPath.size()); i++) {
                unsigned int tmp1, tmp2;
                costmap_->indexToCells(bestPath[i], tmp1, tmp2);
                double x, y;
                costmap_->mapToWorld(tmp1, tmp2, x, y);

                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                plan.push_back(pose);
            }
            plan.push_back(goal);
            publishPlan(plan);
            return true;
    }

    double AstarPlanner::getMoveCost(int firstIndex, int secondIndex)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1,firstYCord = tmp2;
        costmap_->indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;

        int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
        // Error checking
        if(difference != 1 && difference != 2){
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        if(difference == 1)
            return 1.0;
        else
            return 1.414;
    }

    double AstarPlanner::getHeuristic(int cell_index, int goal_index)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;
        //ROS_INFO("distance is %f",max(abs(goalY - startY) , abs(goalX - startX))-min(abs(goalY - startY) , abs(goalX - startX))+1.414*min(abs(goalY - startY) , abs(goalX - startX)));
        //return max(abs(goalY - startY) , abs(goalX - startX))-min(abs(goalY - startY) , abs(goalX - startX))+1.414*min(abs(goalY - startY) , abs(goalX - startX));//absolute tight
        //if((abs(goalY - startY) + abs(goalX - startX))<20){
            //ROS_INFO("distance is %f",max(abs(goalY - startY) , abs(goalX - startX))-min(abs(goalY - startY) , abs(goalX - startX))+1.414*min(abs(goalY - startY) , abs(goalX - startX)));
        //}
        return max(abs(goalY - startY) , abs(goalX - startX))-min(abs(goalY - startY) , abs(goalX - startX))+1.414*min(abs(goalY - startY) , abs(goalX - startX));//absolute tight
    }

    double AstarPlanner::getGoalAngleDifference(const Eigen::Isometry3d& pose,const Eigen::Isometry3d& goal){
        double startX = pose.translation()[0], startY = pose.translation()[1];
        double goalX = goal.translation()[0], goalY = goal.translation()[1];
        double yaw = pose.rotation().eulerAngles(2,1,0)[0];
        double bian = hypot(startX-goalX,startY-goalY);
        double goal_th;
        if(goalY-startY<0){
            goal_th = -acos((goalX-startX)/bian);
        }else
            goal_th = acos((goalX-startX)/bian);
        double angle= fabs(angles::shortest_angular_distance(yaw, goal_th));
        return angle;
    }
    bool AstarPlanner::isInBounds(int x, int y)
    {
        if( x < 0 || y < 0 || x >= height || y >= width)
            return false;
        return true;
    }

    vector<int> AstarPlanner::get_neighbors(int current_cell)
    {
        vector<int> neighborIndexes;

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                unsigned tmp1, tmp2;
                costmap_->indexToCells(current_cell, tmp1, tmp2);
                int nextX = tmp1 + i;
                int nextY = tmp2 + j;
                int nextIndex = costmap_->getIndex(nextX, nextY);
                //not currentnode & isinbound & no obstacle
                if(!( i == 0 && j == 0) && isInBounds(nextX, nextY) && OGM[nextIndex])
                {
                    neighborIndexes.push_back(nextIndex);
                }
            }
        }
        return neighborIndexes;
    }


    void AstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        //create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }
}

// Required for multiset sorting
bool operator <(const Node& x,const Node& y) {
    return (x.gCost+x.hCost) < (y.gCost+y.hCost);
}