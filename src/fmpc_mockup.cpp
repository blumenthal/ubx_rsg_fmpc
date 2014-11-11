#include "fmpc_mockup.hpp"


/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct fmpc_mockup_info
{
        /* add custom block local data here */
        float fenceAABB[4];		// AABB bounding box to constrain the FMPC. [Ax, Ay, Bx, By]
        float obstacle[3];		// Obstacle represented as a circle with the following parameters: [x,y,r]
        float goal[2];		// A new goal setpoint represented as a point: [x,y]. Theta (rotation) is not supported.
        float robotPose[2];		// The current pose of the robot represented as a point: [x,y]. Theta (rotation) is not supported."

        /* this is to have fast access to ports for reading and writing, without
         * needing a hash table lookup */
        struct fmpc_mockup_port_cache ports;
};

/* init */
int fmpc_mockup_init(ubx_block_t *b)
{
        int ret = -1;
        struct fmpc_mockup_info *inf;

        /* allocate memory for the block local state */
        if ((inf = (struct fmpc_mockup_info*)calloc(1, sizeof(struct fmpc_mockup_info)))==NULL) {
                ERR("fmpc_mockup: failed to alloc memory");
                ret=EOUTOFMEM;
                return -1;
        }
        b->private_data=inf;
        update_port_cache(b, &inf->ports);

        inf->fenceAABB[0] = 0;
        inf->fenceAABB[1] = 0;
        inf->fenceAABB[2] = 0;
        inf->fenceAABB[3] = 0;

        inf->obstacle[0] = 0;
        inf->obstacle[1] = 0;
        inf->obstacle[2] = 0;

        inf->goal[0] = 0;
        inf->goal[1] = 0;

        inf->robotPose[0] = 0;
        inf->robotPose[1] = 0;

        return 0;
}

/* start */
int fmpc_mockup_start(ubx_block_t *b)
{
        /* struct fmpc_mockup_info *inf = (struct fmpc_mockup_info*) b->private_data; */
        int ret = 0;
        return ret;
}

/* stop */
void fmpc_mockup_stop(ubx_block_t *b)
{
        /* struct fmpc_mockup_info *inf = (struct fmpc_mockup_info*) b->private_data; */
}

/* cleanup */
void fmpc_mockup_cleanup(ubx_block_t *b)
{
        free(b->private_data);
}

/* step */
void fmpc_mockup_step(ubx_block_t *b)
{
	printf("fmpc_mockup_step\n");
	struct fmpc_mockup_info *inf = (struct fmpc_mockup_info*) b->private_data;
	int32_t readBytes;

	/* read virtual fence data */
	readBytes = read_fmpc_virtual_fence_4(inf->ports.fmpc_virtual_fence, &inf->fenceAABB);
	printf("\tfmpc_mockup: virtual fence; received %i bytes\n",  readBytes);
	printf("\tfmpc_mockup: virtual fence = ( %f, %f, %f, %f)\n", inf->fenceAABB[0], inf->fenceAABB[1], inf->fenceAABB[2], inf->fenceAABB[3]);

	/* read obstacle data */
	readBytes = read_fmpc_obstacle_3(inf->ports.fmpc_obstacle, &inf->obstacle);
	printf("\tfmpc_mockup: obstacle; received %i bytes\n",  readBytes);
	printf("\tfmpc_mockup: obstacle = ( %f, %f, %f)\n", inf->obstacle[0], inf->obstacle[1], inf->obstacle[2]);

	/* read goal data */
	readBytes = read_fmpc_goal_pose_2(inf->ports.fmpc_goal_pose, &inf->goal);
	printf("\tfmpc_mockup: goal; received %i bytes\n",  readBytes);
	printf("\tfmpc_mockup: goal = ( %f, %f)\n", inf->goal[0], inf->goal[1]);

	/* write current pose or robot */
	inf->robotPose[0] = inf->goal[0]/2.0; // just some values that can change over time...
	inf->robotPose[1] = inf->goal[1]/2.0;
	printf("\tfmpc_mockup: pose = ( %f, %f)\n", inf->robotPose[0], inf->robotPose[1]);
	write_fmpc_robot_pose_2(inf->ports.fmpc_robot_pose, &inf->robotPose);
}

