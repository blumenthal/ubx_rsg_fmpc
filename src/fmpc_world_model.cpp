#include "fmpc_world_model.hpp"

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct fmpc_world_model_info
{
        /* add custom block local data here */

        /* this is to have fast access to ports for reading and writing, without
         * needing a hash table lookup */
        struct fmpc_world_model_port_cache ports;
};

/* init */
int fmpc_world_model_init(ubx_block_t *b)
{
        int ret = -1;
        struct fmpc_world_model_info *inf;

        /* allocate memory for the block local state */
        if ((inf = (struct fmpc_world_model_info*)calloc(1, sizeof(struct fmpc_world_model_info)))==NULL) {
                ERR("fmpc_world_model: failed to alloc memory");
                ret=EOUTOFMEM;
                goto out;
        }
        b->private_data=inf;
        update_port_cache(b, &inf->ports);
        ret=0;
out:
        return ret;
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
        /*
        struct fmpc_world_model_info *inf = (struct fmpc_world_model_info*) b->private_data;
        */
}

