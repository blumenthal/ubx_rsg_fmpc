
#define DEBUG

#include <stdio.h>
#include <stdlib.h>
#include "ubx.h"
#include "types/ptrig_config.h" // (still) part of std_blocks in microblox
#include "types/ptrig_config.h.hexarr"

ubx_port_t motion_coordinator_ports[] = {
    { .name="cmd_in", .in_type_name="unsigned char", .in_data_len=1, .doc="Input data port that toggles the controller on/off."  },
	{ NULL }
};

ubx_config_t motion_coordinator_conf[] = {
    { .name="trig_blocks", .type_name = "struct ptrig_config", .doc = "List of blocks that are coordinated by this block." },
	{ NULL }
};

char motion_coordinator_meta[] =
	"{ doc='A motion coordination block',"
	"  license='BSD-3-Clause',"
	"  real-time=false,"
	"}";

struct motion_coordinator_info {
	  struct ptrig_config* trig_list;			/* similar as in ptrig block */
	  unsigned int trig_list_len;
};

def_read_fun(read_command, unsigned char)


static int motion_coordinator_init(ubx_block_t *b)
{
	int ret=0;

	DBG(" ");
	if ((b->private_data = calloc(1, sizeof(struct motion_coordinator_info)))==NULL) {
		ERR("Failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}


	struct motion_coordinator_info *inf = b->private_data;

	/* Get list of blocks that are coordinated */
	ubx_data_t* trig_list_data = 0;
	trig_list_data = ubx_config_get_data(b, "trig_blocks");
	if (trig_list_data != 0) {
		inf->trig_list = (struct ptrig_config*)trig_list_data->data;
		inf->trig_list_len = 2u; //(unsigned int)trig_list_data->len;
		printf("motion_coordinator: has %i blocks thet are beeind coordinated.", inf->trig_list_len);
	} else {
		ret=-1;
		goto out;
	}


 out:
	return ret;
}

static int motion_coordinator_start(ubx_block_t *b)
{
	printf("motion_coordinator_start\n");
	return 0;
}


static void motion_coordinator_step(ubx_block_t *b)
{
	printf("motion_coordinator_step\n");
	unsigned char cmd; // A complete "command message"
	int32_t readBytes = 0;
	int idx=0;

	struct motion_coordinator_info* inf = (struct motion_coordinator_info*)(b->private_data);

	ubx_port_t* cmd_port = ubx_port_get(b, "cmd_in");
	readBytes = read_command(cmd_port, &cmd);

	if (readBytes == 1) {
		switch (cmd) {
			case 0x00:

				printf("motion_coordinator: OFF\n");
				for(idx=0; idx<inf->trig_list_len; idx++) {
					ubx_block_stop(inf->trig_list[idx].b);
				}
				break;

			case 0x01:

				printf("motion_coordinator: ON\n");
				for(idx=0; idx<inf->trig_list_len; idx++) {
					ubx_block_start(inf->trig_list[idx].b);
				}
				break;

			default:

				printf("motion_coordinator: unknown commnad");
				break;
		}
	}



}

static void motion_coordinator_stop(ubx_block_t *b)
{

}

static void motion_coordinator_cleanup(ubx_block_t *b)
{
	free(b->private_data);
}


/* put everything together */
ubx_block_t motion_coord = {
	.name = "motion_coordinator",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = motion_coordinator_meta,
	.configs = motion_coordinator_conf,
	.ports = motion_coordinator_ports,

	/* ops */
	.init = motion_coordinator_init,
	.start = motion_coordinator_start,
	.step = motion_coordinator_step,
	.stop = motion_coordinator_stop,
	.cleanup = motion_coordinator_cleanup,
};

static int motion_coodinatior_mod_init(ubx_node_info_t* ni)
{
	return ubx_block_register(ni, &motion_coord);
}

static void motion_coodinatior_mod_a_cleanup(ubx_node_info_t *ni)
{
	ubx_block_unregister(ni, "motion_coordinator");
}

UBX_MODULE_INIT(motion_coodinatior_mod_init)
UBX_MODULE_CLEANUP(motion_coodinatior_mod_a_cleanup)
UBX_MODULE_LICENSE_SPDX(BSD-3-Clause)
