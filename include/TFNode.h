/*
 * TFNode.h
 *
 *  Created on: Jul 6, 2011
 *      Author: amigo
 */

#ifndef TFNODE_H_
#define TFNODE_H_

struct tf_info {
    ros::Time first_timestamp;
    ros::Time timestamp;
	string broadcaster;
    int num_received;
};

class TFNode {

protected:
	map<TFNode*, tf_info> parents_;
	map<TFNode*, tf_info> children_;
	string frame_id_;

public:


	TFNode(string id) {
		frame_id_ = id;
	}

    bool addChild(TFNode* child, const ros::Time& timestamp) {
        tf_info& info = children_[child];
        info.timestamp = timestamp;
        if (info.first_timestamp == ros::Time()) {
            info.first_timestamp = timestamp;
        } else {
            info.num_received++;
        }

		child->parents_[this] = info;
		return true;
	}

	const map<TFNode*, tf_info>& getParents() const {
		return parents_;
	}

	const map<TFNode*, tf_info>& getChildren() const {
		return children_;
	}


	string getFrameID() const {
		return frame_id_;
	}
};

#endif /* TFNODE_H_ */
