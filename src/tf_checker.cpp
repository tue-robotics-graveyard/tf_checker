/*
 * tf_checker.cpp
 *
 *  Created on: Jul 6, 2011
 *      Author: amigo
 */

// ros
#include "ros/ros.h"

#include "tf/tfMessage.h"
#include "geometry_msgs/TransformStamped.h"

#include <map>
#include <list>


using namespace std;

#include "TFNode.h"

//map<string, map<string, tf_info> > tf_map_;

set<TFNode*> roots_;
map<string, TFNode*> node_map_;

/*
bool getTransformInfo(string parent_frame, string child_frame, tf_info& info) {
	map<string, map<string, tf_info> >::iterator it_map = tf_map_.find(parent_frame);
	if (it_map != tf_map_.end()) {
		map<string, tf_info>::iterator it_map2 = (*it_map).second.find(child_frame);
		if (it_map2 != (*it_map).second.end()) {
			info = (*it_map2).second;
			return true;
		}
	}
	return false;
}
*/

TFNode* findOrCreateNode(string frame_id) {
	TFNode* node = 0;
	map<string, TFNode*>::iterator it_map = node_map_.find(frame_id);
	if (it_map == node_map_.end()) {
		// node was not yet known, so create it
		node = new TFNode(frame_id);
		node_map_[frame_id] = node;
		// every node starts as a root
		roots_.insert(node);
	} else {
		node = (*it_map).second;
	}
	return node;

}

void tf_callback(const tf::tfMessage::ConstPtr& msg) {
	for(vector<geometry_msgs::TransformStamped>::const_iterator it = msg->transforms.begin(); it != msg->transforms.end(); ++it) {
		const geometry_msgs::TransformStamped& tf = (*it);

		// find nodes corrsponding to parent and child frame, and create if they do not exist
		TFNode* parent_node = findOrCreateNode(tf.header.frame_id);
		TFNode* child_node = findOrCreateNode(tf.child_frame_id);

		// add child frame to node
        parent_node->addChild(child_node, tf.header.stamp);

		// the child node is not a root anymore
		roots_.erase(child_node);

		// check if reverse transform was already known. If so, print error:
		//tf_info rev_info;
		//if (getTransformInfo(tf.child_frame_id, tf.header.frame_id, rev_info)) {
		//	ROS_ERROR_STREAM("SYMMETRICAL TRANSFORM RELATION FOUND: " << tf.child_frame_id << " <-> " << tf.header.frame_id);
		//}

	}

}

void printTransforms() {

    ros::Time t_min = ros::TIME_MAX;
    ros::Time t_max = ros::TIME_MIN;

    ROS_INFO_STREAM("*********************** TRANSFORMS ************************");
	for(map<string, TFNode*>::iterator it = node_map_.begin(); it != node_map_.end(); ++it) {
		TFNode* parent_node = (*it).second;
		const map<TFNode*, tf_info>& children = parent_node->getChildren();
		for(map<TFNode*, tf_info>::const_iterator it2 = children.begin(); it2 != children.end(); ++it2) {
            const tf_info& info = it2->second;

            double freq = (double)info.num_received / (info.timestamp - info.first_timestamp).toSec();

            ROS_INFO_STREAM("   " << parent_node->getFrameID() << " -> " << (*it2).first->getFrameID() << ", last update = " << info.timestamp
                    << ", freq = " << freq);
            if (info.timestamp < t_min) {
                t_min = info.timestamp;
            }
            if (info.timestamp > t_max) {
                t_max = info.timestamp;
            }
		}
	}
	ROS_INFO_STREAM("");

	if (roots_.size() == 1) {
		ROS_INFO_STREAM("ROOT: " << (*roots_.begin())->getFrameID() << endl);
	} else	if (roots_.size() > 1) {
		ROS_ERROR_STREAM("MULTIPLE ROOTS:");
		for(set<TFNode*>::iterator it = roots_.begin(); it != roots_.end(); ++it) {
			ROS_ERROR_STREAM("   - " << (*it)->getFrameID());
		}
		ROS_ERROR_STREAM("");
    }

    ros::Duration t_gap = t_max - t_min;
    if (t_gap < ros::Duration(1.0)) {
        ROS_INFO_STREAM("Largest time gap: " << t_gap << " seconds");
    } else {
        ROS_ERROR_STREAM("SIGNIFICANT TIME GAP: " << t_gap << " seconds");
    }


}

void checkMultipleParents() {
	for(map<string, TFNode*>::iterator it = node_map_.begin(); it != node_map_.end(); ++it) {
		TFNode* node = (*it).second;

		const map<TFNode*, tf_info>& parents = node->getParents();
		if (parents.size() > 1) {
			ROS_ERROR_STREAM("MULTIPLE PARENTS: child " << node->getFrameID() << " has parents:");
			for(map<TFNode*, tf_info>::const_iterator it2 = parents.begin(); it2 != parents.end(); ++it2) {
				ROS_ERROR_STREAM("   - " << (*it2).first->getFrameID() << ", last update: " << (*it2).second.timestamp);
			}
		}
	}
}

void checkInconsistencies() {
	map<TFNode*, bool> visited;

	// set all nodes to unvisited
	for(map<string, TFNode*>::iterator it = node_map_.begin(); it != node_map_.end(); ++it) {
		visited[(*it).second] = false;
	}

	for(set<TFNode*>::iterator it = roots_.begin(); it != roots_.end(); ++it) {

	}



}

int main(int argc, char **argv) {
	// Initialize node
	ros::init(argc, argv, "tf_checker");
	ros::NodeHandle n("~");

	//int pruning_depth = 2;
	//n.getParam("pruning_depth", pruning_depth);

	// Subscribe to the object detector
	ros::Subscriber sub = n.subscribe("/tf", 1000, &tf_callback);

	// Publisher
	//pub_wm = n.advertise<object_msgs::SeenObjectArray>("/seen_object_array", 100);

	int rate = 20;
	ros::Rate r(rate);
	int count = 0;

	while(n.ok()) {
		ros::spinOnce();

		++count;
		if (count == 20) {
			printTransforms();
			checkMultipleParents();
			count = 0;
		}

		r.sleep();

	}


	return 0;


}
