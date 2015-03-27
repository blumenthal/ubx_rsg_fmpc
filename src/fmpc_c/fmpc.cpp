/*
 * A demo C++ block
 */

#ifndef DEBUG
#define DEBUG 1
#endif

#include <iostream>
using namespace std;

#include "ubx.h"
#include <cstring>

#include <string.h>
#include <cstdlib>
#include <iostream>
#include "mol.hpp"
#include "fmpc_solver_top.hpp"
#include "../youbot_driver/types/youbot_base_motorinfo.h"
# include <types/kdl.h> // this has moved to the UBX_MODULES

#include "types/fmpc_config.h"
#include "types/fmpc_config.h.hexarr"

using namespace std;
using namespace MOL;
//#define FMPC_HW
#ifdef FMPC_HW
#define XFmpc_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XFmpc_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))
#endif


typedef union{
        unsigned int u;
        float f;
} float_union;

void youbot_fmpc(struct fmpc_info* inf, struct kdl_frame *fmpc_odom_frame, struct kdl_twist *fmpc_odom_twist, struct kdl_twist *cmd_twist, int32_t *cmd_vel);
#ifdef FMPC_HW
void Set_Value(XFmpc_solver_top *InstancePtr,u32 BaseAddress,int size,float *data){
        int i=0;
        float_union temp;
        for(i=0;i<size;i++){
                temp.f = *(data+i);
                XFmpc_WriteReg(InstancePtr->Axi4lites_BaseAddress,BaseAddress + 8*i,temp.u);
        }
}

void Get_Value(XFmpc_solver_top *InstancePtr,u32 BaseAddress,int size,float *data){
        int i=0;
        float_union temp;
        for(i=0;i<size;i++){
                temp.u = XFmpc_ReadReg(InstancePtr->Axi4lites_BaseAddress,BaseAddress+8*i);
                *(data+i) = temp.f;
        }
}
#endif
#define PRINT_VAR
#define MAX_A_VAL 0.4
#define MAX_A_VAL2 0.4
#define Q_VAL1 0.0010
#define Q_VAL2 0.0010
type_f Q_a_2[FMPC_GN*FMPC_GN]={
10*Q_VAL1,   0.00000,   0.00000,   0.00000,
0.00000,   10*Q_VAL1,   0.00000,   0.00000,
0.00000,   0.00000,   Q_VAL2,   0.00000,
0.00000,   0.00000,   0.00000,   Q_VAL2

};

//#define KAPPA 1e-3
//#define NITER 5
//#define RADIUS_R 0.5
//#define X_POS_OFFSET 5
#define YOUBOT_NR_OF_WHEELS 4
//#define UMAX_C 0.39195//tn*6*Kt*eff //0.39195 //3.484
//#define UMIN_C -UMAX_C//-0.39195 //-3.484

#define XMAX_LEN 40
#define XMIN_LEN 40
#define UMAX_LEN 40
#define UMIN_LEN 40
#define X_LEN 40
#define U_LEN 40
#define X_ARRAY_LEN 4
#define Q_LEN 16

float tempW[4];
float eff=0.60;
float tn=26;
float Kt=0.0335;
float Vmax=24;
float Ra=1.03;
float invJ[8]={
-19.231,   19.231,
 19.231,   19.231,
-19.231,  -19.231,
 19.231,  -19.231
};

//type_f Xmax_const[FMPC_GN]={5,5,MAX_A_VAL,MAX_A_VAL2};
//type_f Xmin_const[FMPC_GN]={-5,-5,-MAX_A_VAL,-MAX_A_VAL2};
//type_f Umax_const[FMPC_GN]={UMAX_C,UMAX_C,UMAX_C,UMAX_C};
//type_f Umin_const[FMPC_GN]={UMIN_C,UMIN_C,UMIN_C,UMIN_C};
#define X_AO_LEN 40
#define U_AO_LEN 40

ubx_type_t fmpc_config_type = def_struct_type(struct fmpc_config, &fmpc_config_h);
/* function block meta-data
 * used by higher level functions.
 */

char fmpc_meta[] =
	"{ doc='A fmpc number generator function block',"
	"  license='LGPL',"
	"  real-time=true,"
	"}";

/* configuration
 * upon cloning the following happens:
 *   - value.type is resolved
 *   - value.data will point to a buffer of size value.len*value.type->size
 *
 * if an array is required, then .value = { .len=<LENGTH> } can be used.
 */
ubx_config_t fmpc_config[] = {
/*	{ .name="param_kappa", 		.type_name = "float" },
	{ .name="param_iteration",	.type_name = "int" },
	{ .name="param_fence",		.type_name = "float",	.value={.len=4} },
	{ .name="param_states_max",	.type_name = "float",	.value={.len=4} },
	{ .name="param_states_min",	.type_name = "float",	.value={.len=4} },
	{ .name="param_state_init",	.type_name = "float",	.value={.len=4} },
	{ .name="param_inputs_max",	.type_name = "float",	.value={.len=4} },
	{ .name="param_inputs_min",	.type_name = "float",	.value={.len=4} },
	{ .name="param_inputs_init",	.type_name = "float",	.value={.len=4} },
	{ .name="param_obstacle",	.type_name = "float",	.value={.len=3} },
*/	
	{.name="fmpc_config", .type_name="struct fmpc_config"},	
	{ NULL },
};


ubx_port_t fmpc_ports[] = {
	{ .name="cmd_vel", .out_type_name="int32_t", .out_data_len=4},
	{ .name="cmd_twist", .out_type_name="struct kdl_twist"},
	{ .name="motor_info", .in_type_name="struct youbot_base_motorinfo"},
        { .name="fmpc_odom_port", .in_type_name="struct kdl_frame" },
        { .name="fmpc_twist_port", .in_type_name="struct kdl_twist" },
	{ .name="youbot_info_port", .out_type_name="char", .out_data_len=256},
	{ .name="fmpc_virtual_fence", .in_type_name="float", .in_data_len=4},
	{ .name="fmpc_obstacle", .in_type_name="float", .in_data_len=3},
	{ .name="fmpc_goal_pose", .in_type_name="float", .in_data_len=2},
	{ .name="fmpc_robot_pose", .out_type_name="float", .out_data_len=2},
	{ NULL },
};


struct fmpc_info{
	int fd;
	int *p;
	int dummy;
        type_f xmax_a[FMPC_GT*FMPC_GN];
        type_f xmin_a[FMPC_GT*FMPC_GN];
        type_f umax_a[FMPC_GT*FMPC_GM];
        type_f umin_a[FMPC_GT*FMPC_GM];
        type_f X_a[FMPC_GT*FMPC_GN];
        type_f U_a[FMPC_GT*FMPC_GM];
        float x_a[FMPC_GN];
        float U_array_a[FMPC_GM];
	type_f temp_x_a[FMPC_GN];
	float obstacle[3];	
        type_f kappa;
        int niter;
	//type_f init_x[FMPC_GN];
	type_f Xmax_const[FMPC_GN];
	type_f Xmin_const[FMPC_GN];
	float goal_pose[2];
};

	XFmpc_solver_top fmpc;
	Matrix<type_f,FMPC_GN,FMPC_GN> A_m;
	Matrix<type_f,FMPC_GN,FMPC_GM> B_m;
	Matrix<type_f,FMPC_GN,FMPC_GN> Q_m;
	Matrix<type_f,FMPC_GM,FMPC_GM> R_m;
	Matrix<type_f,FMPC_GN,FMPC_GN> Qf_m;
	Matrix<type_f,FMPC_GN,1> x_m;
	Matrix<type_f,FMPC_GT,FMPC_GN> X_m;
	Matrix<type_f,FMPC_GT,FMPC_GM> U_m;

	Matrix<type_f,1,FMPC_GM> ut;

	type_f A_a[FMPC_GN*FMPC_GN];
	type_f B_a[FMPC_GN*FMPC_GM];
	type_f Q_a[FMPC_GN*FMPC_GN];
	type_f R_a[FMPC_GM*FMPC_GM];
	type_f Qf_a[FMPC_GN*FMPC_GN];
	type_f X_ao[FMPC_GT*FMPC_GN];
	type_f U_ao[FMPC_GT*FMPC_GM];
	type_f X_a_next[FMPC_GT*FMPC_GN];
	type_f U_a_next[FMPC_GT*FMPC_GM];
	
/*convenience functions to read/write from the ports*/
def_read_fun(read_motorinfo, struct youbot_base_motorinfo)
def_read_fun(read_kdl_frame, struct kdl_frame)
def_read_fun(read_kdl_twist, struct kdl_twist)
def_write_arr_fun(write_int4, int32_t,4);
def_write_fun(write_kdl_twist, struct kdl_twist)
def_write_arr_fun(write_char256, char, 256);
def_read_arr_fun(read_float3, float, 3);
def_write_arr_fun(write_float2, float, 2);
def_read_arr_fun(read_float2, float, 2);

static int fmpc_init(ubx_block_t *c)
{
	int ret=0;	
	DBG("fmpc init start");
        if ((c->private_data = calloc(1, sizeof(struct fmpc_info)))==NULL) {
                ERR("Failed to alloc memory");
                ret=EOUTOFMEM;
                return ret;
        }

        struct fmpc_info* inf = (struct fmpc_info*) c->private_data;

        unsigned int clen;
        struct fmpc_config* fmpc_conf;
        fmpc_conf = (struct fmpc_config*) ubx_config_get_data_ptr(c, "fmpc_config", &clen);
	


#ifdef FMPC_HW
        inf->fd=open("/dev/mem", O_RDWR);
        if(inf->fd == -1) {
        printf("Err: cannot open /dev/mem\n");
        return -1;
        }
        inf->p = (int *)mmap(0, 65536, PROT_READ|PROT_WRITE, MAP_SHARED, inf->fd, 0x43c00000);
        fmpc.Axi4lites_BaseAddress=(unsigned int)inf->p;
#endif

	std::string data_dir( FMPC_DATA_DIR ); //retrives durin compile time; typically points to ../share/ubx/data
	std::string filename;

	filename = data_dir + "/data_fmpc/A.txt";
    LoadMatrixFromFile(A_m, filename.c_str());
	filename = data_dir + "/data_fmpc/B.txt";
	LoadMatrixFromFile(B_m, filename.c_str());
	filename = data_dir + "/data_fmpc/Q.txt";
	LoadMatrixFromFile(Q_m, filename.c_str());
	filename = data_dir + "/data_fmpc/R.txt";
	LoadMatrixFromFile(R_m, filename.c_str());
	filename = data_dir + "/data_fmpc/Qf.txt";
	LoadMatrixFromFile(Qf_m, filename.c_str());
	filename = data_dir + "/data_fmpc/X0.txt";
	LoadMatrixFromFile(X_m, filename.c_str());
	filename = data_dir + "/data_fmpc/U0.txt";
	LoadMatrixFromFile(U_m, filename.c_str());

//    LoadMatrixFromFile(A_m, "/tmp/data_fmpc/A.txt");
//	LoadMatrixFromFile(B_m, "/tmp/data_fmpc/B.txt");
//	LoadMatrixFromFile(Q_m, "/tmp/data_fmpc/Q.txt");
//	LoadMatrixFromFile(R_m, "/tmp/data_fmpc/R.txt");
//	LoadMatrixFromFile(Qf_m, "/tmp/data_fmpc/Qf.txt");
//	LoadMatrixFromFile(X_m, "/tmp/data_fmpc/X0.txt");
//	LoadMatrixFromFile(U_m, "/tmp/data_fmpc/U0.txt");

	Matrix2array(A_m,A_a);
	Matrix2array(B_m,B_a);
	Matrix2array(Q_m,Q_a);
	Matrix2array(R_m,R_a);
	Matrix2array(Qf_m,Qf_a);

	Matrix2array(X_m,inf->X_a);
	Matrix2array(U_m,inf->U_a);

	float param_states_max_arr[4];
	float param_states_min_arr[4];
	float param_inputs_max_arr[4];
	float param_inputs_min_arr[4];
	
	for(int i=0;i<4;i++){
	param_states_max_arr[i]=fmpc_conf->param_states_max[i];
	param_states_min_arr[i]=fmpc_conf->param_states_min[i];
	param_inputs_max_arr[i]=fmpc_conf->param_inputs_max[i];
	param_inputs_min_arr[i]=fmpc_conf->param_inputs_min[i];
	inf->Xmax_const[i]=fmpc_conf->param_states_max[i];
	inf->Xmin_const[i]=fmpc_conf->param_states_min[i];
	inf->x_a[i]=fmpc_conf->param_states_init[i];
	//inf->init_x[i]=fmpc_conf->param_states_init[i];
	}
	for(int i=0;i<3;i++)
		inf->obstacle[i]=fmpc_conf->param_obstacle[i];
	for(int i=0;i<FMPC_GT;i++){
		for(int j=0;j<FMPC_GN;j++){
			inf->xmax_a[i*FMPC_GN+j]=param_states_max_arr[j];
			inf->xmin_a[i*FMPC_GN+j]=param_states_min_arr[j];
		}
	}
	for(int i=0;i<FMPC_GT;i++){
		for(int j=0;j<FMPC_GM;j++){
			inf->umax_a[i*FMPC_GM+j]=param_inputs_max_arr[j];
			inf->umin_a[i*FMPC_GM+j]=param_inputs_min_arr[j];
		}
	}

	inf->kappa=fmpc_conf->param_kappa;
	inf->niter=fmpc_conf->param_iteration;
	inf->goal_pose[0]=0;
	inf->goal_pose[1]=0;

#ifdef FMPC_HW
	fmpc_solver_top_assign_variables(&fmpc, (char)0, A_a,B_a,inf->xmax_a,inf->xmin_a,inf->umax_a,inf->umin_a,Q_a,R_a,Qf_a,inf->kappa,inf->niter,inf->X_a,inf->U_a,inf->x_a);
#endif

//	cout << "fmpc_init: hi from " << c->name << endl;
	return 0;
}


static void fmpc_cleanup(ubx_block_t *c)
{
	cout << "fmpc_cleanup: hi from " << c->name << endl;
}

static int fmpc_start(ubx_block_t *c)
{
	cout << "fmpc_start: hi from " << c->name << endl;
	return 0; /* Ok */
}

static void fmpc_step(ubx_block_t *c) {
	int32_t cmd_vel[4];
	int ret;
	struct fmpc_info* inf = (struct fmpc_info*) c->private_data;
	struct kdl_frame fmpc_odom_frame, fmpc_odom_frame_local;
	struct kdl_twist fmpc_twist;
	struct kdl_twist cmd_twist;
        struct youbot_base_motorinfo ymi;
	char data_buf[256];
	float obstacle[3];
	float robot_pose[2];
	float goal_pose[2];
	/* get ports */
	ubx_port_t* p_cmd_vel = ubx_port_get(c, "cmd_vel");
        ubx_port_t* p_cmd_twist = ubx_port_get(c, "cmd_twist");
        ubx_port_t* p_motorinfo = ubx_port_get(c, "motor_info");
	ubx_port_t* p_youbot_info = ubx_port_get(c, "youbot_info_port");
	
	ubx_port_t* p_obstacle_info = ubx_port_get(c, "fmpc_obstacle");        
	ubx_port_t* p_fmpc_robot_pose = ubx_port_get(c, "fmpc_robot_pose");        
	ubx_port_t* p_fmpc_goal_pose = ubx_port_get(c, "fmpc_goal_pose");
	
	/* read new motorinfo */
        read_motorinfo(p_motorinfo, & ymi);

        /* do something here */
        inf->dummy++;
        ubx_port_t* fmpc_odom_port = ubx_port_get(c, "fmpc_odom_port");
        ubx_port_t* fmpc_twist_port = ubx_port_get(c, "fmpc_twist_port");
        /*mode*/
        /*
	printf("youbot twist read returned: %d, youbot odom read returned: %d\n",
              read_kdl_twist(fmpc_twist_port, &fmpc_twist), 
                      read_kdl_frame(fmpc_odom_port, &fmpc_odom_frame));
	*/

	read_float2(p_fmpc_goal_pose, &goal_pose);	
	inf->goal_pose[0]=goal_pose[0];
	inf->goal_pose[1]=goal_pose[1];
	ret = read_float3(p_obstacle_info, &obstacle);
	if(ret>0){
		inf->obstacle[0]=obstacle[0]-goal_pose[0];
		inf->obstacle[1]=obstacle[1]-goal_pose[1];
	}
	
	printf("goal_pose: %f, %f\n", goal_pose[0], goal_pose[1]);
	printf("obs_pose: %f, %f, %f\n", obstacle[0], obstacle[1],obstacle[2]);
        
	if(read_kdl_frame(fmpc_odom_port, &fmpc_odom_frame)==1 && read_kdl_frame(fmpc_odom_port, &fmpc_odom_frame_local)==1
        && read_kdl_twist(fmpc_twist_port, &fmpc_twist)==1){
		fmpc_odom_frame_local.p.x = fmpc_odom_frame.p.x - goal_pose[0];
		fmpc_odom_frame_local.p.y = fmpc_odom_frame.p.y - goal_pose[1];

        	youbot_fmpc(inf, &fmpc_odom_frame_local, &fmpc_twist, &cmd_twist, cmd_vel);
	} else {
		printf("read_kdl_frame failed\n");
	}

	//robot_pose[0]=fmpc_odom_frame.p.x+inf->init_x[0];
	//robot_pose[1]=fmpc_odom_frame.p.y+inf->init_x[1];

	robot_pose[0]=fmpc_odom_frame.p.x;
	robot_pose[1]=fmpc_odom_frame.p.y;

	printf("robot_pose: %f, %f\n", robot_pose[0], robot_pose[1]);
	write_float2(p_fmpc_robot_pose, &robot_pose);
	
	/* write out new velocity */
	write_int4(p_cmd_vel, &cmd_vel);
        write_kdl_twist(p_cmd_twist, &cmd_twist);
	
	sprintf(data_buf, "%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,\
		%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f,\
		%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f,\
		%8.3f, %8.3f, %8.3f, %8.3f, %8.3f, %8.3f,\
		%8.3f,%8.3f\n", 
		fmpc_odom_frame.p.x,//+inf->init_x[0], 
		fmpc_odom_frame.p.y,//+inf->init_x[1], 
		fmpc_odom_frame.p.z, 
		fmpc_twist.vel.x, 
		fmpc_twist.vel.y,
		fmpc_twist.vel.z,
		inf->xmax_a[0],
		inf->xmax_a[1],
		inf->xmin_a[0],
		inf->xmin_a[1],
		inf->obstacle[0],
		inf->obstacle[1],
		inf->obstacle[2],
		inf->X_a[0],inf->X_a[1],inf->X_a[4],inf->X_a[5],inf->X_a[8],inf->X_a[9],
		inf->X_a[12],inf->X_a[13],inf->X_a[16],inf->X_a[17],inf->X_a[20],inf->X_a[21],
		inf->X_a[24],inf->X_a[25],inf->X_a[28],inf->X_a[29],inf->X_a[32],inf->X_a[33],
		inf->X_a[36],inf->X_a[37]
		);
	write_char256(p_youbot_info,&data_buf);
	//printf("%f,%f,%f\n",cmd_twist.vel.x, cmd_twist.vel.y, cmd_twist.rot.z);
	//printf("%f,%f,%f\n",fmpc_twist.vel.x,fmpc_twist.vel.y, fmpc_twist.rot.z);
}


/* put everything together */
ubx_block_t fmpc_comp = {
	.name = "fmpc/fmpc",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = fmpc_meta,
	.configs = fmpc_config,
	.ports = fmpc_ports,

	/* ops */
	.init = fmpc_init,
	.start = fmpc_start,
	.step = fmpc_step,
	.cleanup = fmpc_cleanup,
};

static int fmpc_init(ubx_node_info_t* ni)
{
	DBG("fmpc_init 0");
        ubx_type_register(ni, &fmpc_config_type);
	return ubx_block_register(ni, &fmpc_comp);
}

static void fmpc_cleanup(ubx_node_info_t *ni)
{
	DBG(" ");
        ubx_type_unregister(ni, "struct fmpc_config");
	ubx_block_unregister(ni, "fmpc/fmpc");
}

float cal_passover_side(float x1, float y1, float x2, float y2, float xp, float yp){
	float ret;
		ret=(x2-x1)*(yp-y1)-(y2-y1)*(xp-x1);
		//if(x2<x1)
		//	ret=-ret;
		//printf("x1 %f,y1 %f,x2 %f,y2 %f,xp %f,yp %f\n",x1,y1,x2,y2,xp,yp);
	return ret;

}


void youbot_fmpc(struct fmpc_info* inf, struct kdl_frame *fmpc_odom_frame, struct kdl_twist *fmpc_odom_twist, struct kdl_twist *cmd_twist, int32_t *cmd_vel){
/*@param: fmpc_odom: odom calculated from motorinfo
 *@param: p_cmd_vel: velocity set points                
 */
        int j,k;
/*
#ifdef FMPC_SIMULATION
        Get_Value(&fmpc,XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_AO_0_DATA+32,FMPC_GN,inf->x_a);
#else
*/
        inf->x_a[0]=fmpc_odom_frame->p.x;//+inf->init_x[0];//-X_POS_OFFSET;
        inf->x_a[1]=fmpc_odom_frame->p.y;//+inf->init_x[1];
        inf->x_a[2]=fmpc_odom_twist->vel.x;
        inf->x_a[3]=fmpc_odom_twist->vel.y;
#ifdef PRINT_VAR
        printf("Read odom & twist:");
        for(j=0;j<4;j++)
                printf("X[%d]=%6.4f\t",j,inf->x_a[j]);
        //printf("\n");
#endif
//#endif
#ifdef FMPC_HW
        Get_Value(&fmpc,XFMPC_SOLVER_TOP_AXI4LITES_ADDR_U_AO_0_DATA+32,FMPC_GM,inf->U_array_a);
#else
	for(int i=0;i<FMPC_GM;i++)
		inf->U_array_a[i]=U_ao[FMPC_GM+i];
#endif

        for(j=0;j<YOUBOT_NR_OF_WHEELS;j++){
#ifdef FMPC_SIMULATION
                *(cmd_vel+j)=0;
#else
                *(cmd_vel+j)=(int)(inf->U_array_a[j]/0.0335/26*1000);
#endif
        }
    /*
        Rebuild X_a and U_a
    */
#define SKIP 1
#ifdef FMPC_HW
        Get_Value(&fmpc,XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_AO_0_DATA+32*SKIP,X_AO_LEN-4*SKIP,inf->X_a);
        Get_Value(&fmpc,XFMPC_SOLVER_TOP_AXI4LITES_ADDR_U_AO_0_DATA+32*SKIP,U_AO_LEN-4*SKIP,inf->U_a);
#else
	for(int i=SKIP;i<FMPC_GT;i++){
		for(int j=0;j<FMPC_GM;j++){
			inf->U_a[(i-SKIP)*FMPC_GM+j]=U_ao[i*FMPC_GM+j];
		}
		for(int j=0;j<FMPC_GN;j++){
			inf->X_a[(i-SKIP)*FMPC_GN+j]=X_ao[i*FMPC_GN+j];
		}
	}
#endif

k=0;
        for(j=0;j<FMPC_GN;j++)
                inf->X_a[FMPC_GN*(FMPC_GT-k)-1-j]=0;
        for(j=0;j<FMPC_GM;j++)
                inf->U_a[FMPC_GN*(FMPC_GT-k)-1-j]=0;

    /*
        Collision Avoidance:
        xmin((j-1)*n+2) = sqrt(r^2-(X(1,j)-px)^2)+py;
        or xmin(j*n+1:j*n+4)=Xmin;
    */

	float factor=1.0;
	//int flag=1;
        for(j=0;j<FMPC_GT-1;j++){
                if((inf->X_a[j*FMPC_GN]-inf->obstacle[0])*(inf->X_a[j*FMPC_GN]-inf->obstacle[0])+(inf->X_a[j*FMPC_GN+1]-inf->obstacle[1])*(inf->X_a[j*FMPC_GN+1]-inf->obstacle[1])<inf->obstacle[2]*inf->obstacle[2]){
                	
			float s0=0.0;
			s0=cal_passover_side(inf->X_a[j*FMPC_GN],inf->X_a[j*FMPC_GN+1],0,0,inf->obstacle[0],inf->obstacle[1]);
		
			printf("s0=%f",s0);	
			
			if(s0>0){
				//factor=1.0;
				//flag=0;
			inf->xmax_a[j*4+1]=-sqrt(inf->obstacle[2]*inf->obstacle[2]-(inf->X_a[j*FMPC_GN]-inf->obstacle[0])*(inf->X_a[j*FMPC_GN]-inf->obstacle[0]))+inf->obstacle[1]*factor;
			inf->xmax_a[j*4]=-sqrt(inf->obstacle[2]*inf->obstacle[2]-(inf->X_a[j*FMPC_GN+1]-inf->obstacle[1])*(inf->X_a[j*FMPC_GN+1]-inf->obstacle[1]))+inf->obstacle[0]*(-factor);
			//inf->xmax_a[j*4+1]=inf->xmin_a[j*4+1];
			//printf("above? inf->xmax_a[x,y]=%f,%f\n", inf->xmax_a[j*4], inf->xmax_a[j*4+1]);
			}
			else{
				//factor=1.0;        
				//flag=1;
			inf->xmin_a[j*4+1]=sqrt(inf->obstacle[2]*inf->obstacle[2]-(inf->X_a[j*FMPC_GN]-inf->obstacle[0])*(inf->X_a[j*FMPC_GN]-inf->obstacle[0]))+inf->obstacle[1]*factor;
			inf->xmin_a[j*4]=sqrt(inf->obstacle[2]*inf->obstacle[2]-(inf->X_a[j*FMPC_GN+1]-inf->obstacle[1])*(inf->X_a[j*FMPC_GN+1]-inf->obstacle[1]))+inf->obstacle[0]*(-factor);
                        //inf->xmin_a[j*4+1]=inf->xmax_a[j*4+1];
			//printf("underneath? inf->xmin_a(x,y)=%f,%f\n", inf->xmin_a[j*4],inf->xmin_a[j*4+1]);
			inf->xmin_a[j*4]=inf->Xmin_const[0];
			}		
		
			//inf->xmin_a[j*4+1]=sqrt(inf->obstacle[2]*inf->obstacle[2]-(inf->X_a[j*FMPC_GN]-inf->obstacle[0])*(inf->X_a[j*FMPC_GN]-inf->obstacle[0]))+inf->obstacle[1];
		printf("yes!!\n");
                }
                else{
                        for(k=0;k<FMPC_GN;k++){
                                inf->xmin_a[(j+1)*FMPC_GN+k]=inf->Xmin_const[k];
                                inf->xmax_a[(j+1)*FMPC_GN+k]=inf->Xmax_const[k];
                        }

                }

        }

//Limit the torque
        for(j=0;j<FMPC_GT;j++){
                for(k=0;k<4;k++){
                tempW[k]=invJ[k*2]*inf->X_a[j*FMPC_GN+2]+invJ[k*2+1]*inf->X_a[j*FMPC_GN+3];
}
                for(k=0;k<4;k++){
                inf->umax_a[j*FMPC_GN+k] = eff*tn*Kt*(Vmax - Kt*tn*tempW[k]/Ra);
                inf->umin_a[j*FMPC_GN+k] = eff*tn*Kt*(-Vmax - Kt*tn*tempW[k]/Ra);
                }
        }

#ifdef FMPC_HW
    Set_Value(&fmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_XMIN_A_0_DATA, XMIN_LEN, inf->xmin_a);
    Set_Value(&fmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_XMAX_A_0_DATA, XMIN_LEN, inf->xmax_a);
    Set_Value(&fmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_UMIN_A_0_DATA, UMIN_LEN, inf->umin_a);
    Set_Value(&fmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_UMAX_A_0_DATA, UMAX_LEN, inf->umax_a);


    Set_Value(&fmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_A_0_DATA, X_LEN, inf->X_a);
    Set_Value(&fmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_U_A_0_DATA, U_LEN, inf->U_a);
    Set_Value(&fmpc, XFMPC_SOLVER_TOP_AXI4LITES_ADDR_X_A_0_R_DATA, X_ARRAY_LEN, inf->x_a);
#else
    fmpc_solver_top((char)0,A_a,B_a,inf->xmax_a,inf->xmin_a,inf->umax_a,inf->umin_a,Q_a,R_a,Qf_a,inf->kappa,inf->niter,inf->X_a,inf->U_a,inf->x_a,X_ao,U_ao);
#endif

	
        cmd_twist->vel.x = inf->X_a[2];
        cmd_twist->vel.y = inf->X_a[3];


        cmd_twist->vel.z = 0.0;
        cmd_twist->rot.x = 0.0;
        cmd_twist->rot.y = 0.0;
        cmd_twist->rot.z = 0.0;

#ifdef PRINT_VAR
        printf("cmd_twist.vel={%6.4f,%6.4f,0.0000}",cmd_twist->vel.x,cmd_twist->vel.y);
        printf("\n");
#endif
        //usleep(100000);
}

UBX_MODULE_INIT(fmpc_init)
UBX_MODULE_CLEANUP(fmpc_cleanup)
