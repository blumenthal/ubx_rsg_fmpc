#include "fmpc_world_model.hpp"

/* microblx type for the robot scene graph */
#include "types/rsg/types/rsg_types.h"

/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>

using namespace brics_3d::rsg;

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct fmpc_world_model_info
{
        /* add custom block local data here */
        brics_3d::WorldModel* wm;
        brics_3d::rsg::DotVisualizer* wm_printer;

        /* this is to have fast access to ports for reading and writing, without
         * needing a hash table lookup */
        struct fmpc_world_model_port_cache ports;
};

/* init */
int fmpc_world_model_init(ubx_block_t *b)
{
        int ret = -1;
        struct fmpc_world_model_info *inf;

    	/* Configure the logger - default level won't tell us much */
    	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
    	brics_3d::Logger::setLogfile("fmpc_world_model.log");

        /* allocate memory for the block local state */
        if ((inf = (struct fmpc_world_model_info*)calloc(1, sizeof(struct fmpc_world_model_info)))==NULL) {
                ERR("fmpc_world_model: failed to alloc memory");
                ret=EOUTOFMEM;
                return -1;
        }
        b->private_data=inf;
        update_port_cache(b, &inf->ports);

    	unsigned int clen;
    	rsg_wm_handle tmpWmHandle =  *((rsg_wm_handle*) ubx_config_get_data_ptr(b, "wm_handle", &clen));
    	assert(clen != 0);
    	inf->wm = reinterpret_cast<brics_3d::WorldModel*>(tmpWmHandle.wm); // We know that this pointer stores the world model type
    	if(inf->wm == 0) {
    		LOG(FATAL) << " World model handle could not be initialized.";
    		return -1;
    	}

    	/* Attach debug graph printer */
    	inf->wm_printer = new brics_3d::rsg::DotVisualizer(&inf->wm->scene);
    	inf->wm_printer->setFileName("fmpc_graph");
    	inf->wm_printer->setKeepHistory(false);
    	inf->wm->scene.attachUpdateObserver(inf->wm_printer);

        return 0;
}

/* start */
int fmpc_world_model_start(ubx_block_t *b)
{
        /* struct fmpc_world_model_info *inf = (struct fmpc_world_model_info*) b->private_data; */
        int ret = 0;
        return ret;
}

/* stop */
void fmpc_world_model_stop(ubx_block_t *b)
{
        /* struct fmpc_world_model_info *inf = (struct fmpc_world_model_info*) b->private_data; */
}

/* cleanup */
void fmpc_world_model_cleanup(ubx_block_t *b)
{
        free(b->private_data);
}

/* step */
void fmpc_world_model_step(ubx_block_t *b)
{

	struct fmpc_world_model_info *inf = (struct fmpc_world_model_info*) b->private_data;

	/* data to be used for the ports */
	float fenceAABB[4];		// AABB bounding box to constrain the FMPC. [Ax, Ay, Bx, By]
	float obstacle[3];		// Obstacle represented as a circle with the following parameters: [x,y,r]
	float goal[2];		// A new goal setpoint represented as a point: [x,y]. Theta (rotation) is not supported.
	float robotPose[2];		// The current pose of the robot represented as a point: [x,y]. Theta (rotation) is not supported."

	/*
	 * Perform queries and write results to the accordin ports.
	 */

	std::vector<Attribute> queryAttributes;
	vector<Id> resultIds;

	/* Get data and pose of the box */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(Attribute("name", "virtual_fence"));
	inf->wm->scene.getNodes(queryAttributes, resultIds);

	for(vector<Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over groups of scene objects;

		vector<Id> childs;
		inf->wm->scene.getGroupChildren(*it, childs);

		for(vector<Id>::iterator it = childs.begin(); it!=childs.end(); ++it) { // loop over all childs; effectively we take the last found entry

			TimeStamp creationTime;
			Shape::ShapePtr shape;
			inf->wm->scene.getGeometry(*it, shape, creationTime);
			Box::BoxPtr resultBox = boost::dynamic_pointer_cast<Box>(shape);

			if (resultBox != 0) {
				LOG(DEBUG) << "fmpc_wm: Box (x,y,z) = " << resultBox->getSizeX() << " "
						<< resultBox->getSizeY() << " "
						<< resultBox->getSizeZ();

				brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr boxPose;
				inf->wm->scene.getTransformForNode(*it, inf->wm->getRootNodeId(), inf->wm->now(), boxPose);
				LOG(DEBUG) << "fmpc_wm: Pose of box is = " << std::endl << *boxPose;

				const double *matrix = boxPose->getRawData();

				/*  AABB conventiom
				 *
				 *  Ax Ay
				 *  +---------------                -
				 *  |              |                ^
				 *  |       + center c (= pose)   sizeY
				 *  |              |                v
				 *  +---------------                -
				 *                  Bx,By
				 *
				 *  |<-- sizeX --->|
				 *
				 */

				fenceAABB[0] = matrix[brics_3d::matrixEntry::x] - ((resultBox->getSizeX())/2.0); //lop left X
				fenceAABB[1] = matrix[brics_3d::matrixEntry::y] + ((resultBox->getSizeY())/2.0); //top left Y
				fenceAABB[2] = matrix[brics_3d::matrixEntry::x] + ((resultBox->getSizeX())/2.0); // bottom right X
				fenceAABB[3] = matrix[brics_3d::matrixEntry::y] - ((resultBox->getSizeY())/2.0); // bottom right Y
			}
		}
	}

	/* Iff there is a result write to port. */
	if (resultIds.size() > 0) {
		LOG(INFO) << "fmpc_wm: AABB of virtual fence is: (" << fenceAABB[0] << ", " << fenceAABB[1] << ", "<< fenceAABB[2] << ", "<< fenceAABB[3] << ")";
		write_fmpc_virtual_fence_4(inf->ports.fmpc_virtual_fence, &fenceAABB);
	}

	/* Get data of obstacles (spheres) */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(Attribute("name", "obstacle"));
	inf->wm->scene.getNodes(queryAttributes, resultIds);

	for(vector<Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over groups of scene objects;

		vector<Id> childs;
		inf->wm->scene.getGroupChildren(*it, childs);

		for(vector<Id>::iterator it = childs.begin(); it!=childs.end(); ++it) { // loop over all childs; effectively we take the last found entry

			TimeStamp creationTime;
			Shape::ShapePtr shape;
			inf->wm->scene.getGeometry(*it, shape, creationTime);
			Sphere::SpherePtr resultSphere = boost::dynamic_pointer_cast<Sphere>(shape);
			if (resultSphere != 0) {
				LOG(DEBUG) << "fmpc_wm: Sphere (r) = " << resultSphere->getRadius();

				brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr spherePose;
				inf->wm->scene.getTransformForNode(*it, inf->wm->getRootNodeId(), inf->wm->now(), spherePose);
				LOG(DEBUG) << "fmpc_wm: Pose of sphere is = " << std::endl << *spherePose;

				const double *matrix = spherePose->getRawData();

				obstacle[0] = matrix[brics_3d::matrixEntry::x]; // X
				obstacle[1] = matrix[brics_3d::matrixEntry::y]; // Y
				obstacle[2] = resultSphere->getRadius(); 		// radius
			}
		}
	}

	/* Iff there is a result write to port. */
	if (resultIds.size() > 0) {
		LOG(INFO) << "fmpc_wm: Object parameters are (x,y,r): (" << obstacle[0] << ", " << obstacle[1] << ", "<< obstacle[2] << ")";
		write_fmpc_obstacle_3(inf->ports.fmpc_obstacle, &obstacle);
	}


	/* Get data for goal pose */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(Attribute("name", "goal"));
	inf->wm->scene.getNodes(queryAttributes, resultIds);

	for(vector<Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over groups of scene objects;

			brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr goalPose;
			if(inf->wm->scene.getTransformForNode(*it, inf->wm->getRootNodeId(), inf->wm->now(), goalPose)) {
				LOG(DEBUG) << "fmpc_wm: Pose of goal is = " << std::endl << *goalPose;

				const double *matrix = goalPose->getRawData();
				goal[0] = matrix[brics_3d::matrixEntry::x]; // X
				goal[1] = matrix[brics_3d::matrixEntry::y]; // Y
		}
	}

	if (resultIds.size() > 0) {
		LOG(INFO) << "fmpc_wm: Goal parameters are (x,y): (" << goal[0] << ", " << goal[1] << ")";
		write_fmpc_goal_pose_2(inf->ports.fmpc_goal_pose, &goal);
	}

	/*
	 * Update the pose of the robot
	 */
	int32_t readBytes = read_fmpc_robot_pose_2(inf->ports.fmpc_robot_pose, &robotPose);

	if(readBytes == sizeof(robotPose)) {
		LOG(INFO) << "fmpc_wm: New robot pose retrived (x,y): (" << robotPose[0] << ", " << robotPose << ")";

		queryAttributes.clear();
		resultIds.clear();
		queryAttributes.push_back(Attribute("name", "robot"));
		inf->wm->scene.getNodes(queryAttributes, resultIds);

		for(vector<Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over groups of scene objects;

	    	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newRobotPose(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	    			0,1,0,
	    			0,0,1,
	    			robotPose[0],robotPose[1],0)); 	// Translation coefficients
	    	inf->wm->scene.setTransform(*it, newRobotPose, inf->wm->now());
		}
	}
}

