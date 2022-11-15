#include "astar_planner.h"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_planner{
    AstarPlanner::AstarPlanner(){}

    AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            OGM.resize(map_size);


            frame_id_ = costmap_ros->getGlobalFrameID();

            ros::NodeHandle private_nh("~/" + name);

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        OGM.clear();
        for (int i = 0; i < width; i++)
        {
            for (int j = 0; j < height; j++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(j, i));
                //get_cost << cost << endl;
                //cout << "i:, j:" << cost << endl;

                if (cost < 150)
                    OGM[i * width + j] = true;
                else
                    OGM[i * width + j] = false;

            }
        }
        if(!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                 goal.pose.position.x,goal.pose.position.y);
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

        vector<float> gCosts(map_size, infinity);
        vector<int> cameFrom(map_size, -1);

        multiset<Node> priority_costs;

        gCosts[start_index] = 0;

        Node currentNode;
        currentNode.index = start_index;
        currentNode.cost = gCosts[start_index] + 0;
        priority_costs.insert(currentNode);
        bestplan_.clear();
        plan.clear();

        while(!priority_costs.empty())
        {
            // Take the element from the top
            currentNode = *priority_costs.begin();
            //Delete the element from the top
            priority_costs.erase(priority_costs.begin());
            if (currentNode.index == goal_index){
                break;
            }
            // Get neighbors
            vector<int> neighborIndexes = get_neighbors(currentNode.index);

            for(int i = 0; i < neighborIndexes.size(); i++){
                if(cameFrom[neighborIndexes[i]] == -1){
                    Node nextNode;
                    nextNode.index = neighborIndexes[i];
                    unsigned int tx,ty;
                    costmap_->indexToCells(nextNode.index, tx,ty);
                    gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i])+0.2*costmap_->getCost(tx,ty);
                    //nextNode.cost = gCosts[neighborIndexes[i]];    //Dijkstra Algorithm
                    nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                    cameFrom[neighborIndexes[i]] = currentNode.index;
                    priority_costs.insert(nextNode);
                }
            }
        }

        if(cameFrom[goal_index] == -1){
            cout << "Goal not reachable, failed making a global path." << endl;
            return false;
        }

        if(start_index == goal_index)
            return false;
        //Finding the best path
        vector<int> bestPath;
        currentNode.index = goal_index;
        while(currentNode.index != start_index){
            bestPath.push_back(cameFrom[currentNode.index]);
            currentNode.index = cameFrom[currentNode.index];
        }
        reverse(bestPath.begin(), bestPath.end());

        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < bestPath.size(); i++){
            unsigned int tmp1, tmp2;
            costmap_->indexToCells(bestPath[i], tmp1, tmp2);
            double x, y;
            costmap_->mapToWorld(tmp1,tmp2, x, y);

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

            bestplan_.push_back(pose);
        }
        bestplan_.push_back(goal);
        CalcSpline(bestplan_,plan);
        //plan.push_back(goal);
        publishPlan(plan);
        return true;

    }
    bool AstarPlanner::CalcSpline(vector<geometry_msgs::PoseStamped> &bestTraj, vector<geometry_msgs::PoseStamped> &smoothTraj){
        ROS_INFO("Smoothing!!!");
        smoothTraj.clear();
        int division = 3;
        int order = bestTraj.size()/division-1;
//        for(int i = 0; i <= order; i++){
//            ROS_INFO("current x = %f, y = %f",bestTraj[division * i].pose.position.x,bestTraj[division * i].pose.position.y);
//        }
        double dt = 1.0000 / bestTraj.size();
        int seg=22;
        vector<std::pair<double, double>> traj;
        ros::Time plan_time = ros::Time::now();
        if(bestTraj.size()<seg){
        for (int m = 0; m < bestTraj.size(); ++m) {
            pair<double, double> pair;
            pair.first = 0;
            pair.second = 0;
            for (int i = 0; i <= order; i++) {
                //ROS_INFO("%f",m*dt);
                //ROS_INFO("current x = %f, y = %f",bestTraj[division * i].pose.position.x,bestTraj[division * i].pose.position.y);
                pair.first += CalcTerm(order, bestTraj[division * i], i, m*dt).first;
                pair.second += CalcTerm(order, bestTraj[division * i], i, m*dt).second;

            }
            geometry_msgs::PoseStamped p;
            costmap_ros_->getRobotPose(p);

            //ROS_INFO("x = %f, y = %fBZ x = %f, y =%f",bestTraj[m].pose.position.x,bestTraj[m].pose.position.y,pair.first,pair.second);
            geometry_msgs::PoseStamped pose;

            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = pair.first;
            pose.pose.position.y = pair.second;
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            smoothTraj.push_back(pose);
        }
        }else{
            order = seg/division-1;
            for (int m = 0; m < seg; ++m) {
                pair<double, double> pair;
                pair.first = 0;
                pair.second = 0;
                for (int i = 0; i <= order; i++) {
                    //ROS_INFO("%f",m*dt);
                    //ROS_INFO("current x = %f, y = %f",bestTraj[division * i].pose.position.x,bestTraj[division * i].pose.position.y);
                    pair.first += CalcTerm(order, bestTraj[division * i], i, m*dt).first;
                    pair.second += CalcTerm(order, bestTraj[division * i], i, m*dt).second;

                }
                geometry_msgs::PoseStamped pose;

                pose.header.stamp = plan_time;
                pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                pose.pose.position.x = pair.first;
                pose.pose.position.y = pair.second;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                smoothTraj.push_back(pose);
            }
            for (int m = seg; m < bestTraj.size(); ++m) {
                geometry_msgs::PoseStamped pose;

                pose.header.stamp = plan_time;
                pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                pose.pose.position.x = bestTraj[m].pose.position.x;
                pose.pose.position.y = bestTraj[m].pose.position.y;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                smoothTraj.push_back(pose);
            }
        }
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
            return 1.4;
    }

    double AstarPlanner::getHeuristic(int cell_index, int goal_index)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;

        return hypot(goalX - startX,goalY - startY);//abs(goalY - startY) + abs(goalX - startX);
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
};
// Required for multiset sorting
bool operator <(const Node& x, const Node& y) {
    return x.cost < y.cost;
}