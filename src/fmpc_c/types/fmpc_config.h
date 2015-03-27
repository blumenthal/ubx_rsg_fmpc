struct fmpc_config {
        float param_kappa;    
        int   param_iteration;
        float param_fence[4];    
        float param_states_max[4];
        float param_states_min[4];
        float param_states_init[4];
        float param_inputs_max[4];
        float param_inputs_min[4];
        float param_inputs_init[4];
        float param_obstacle[3];
};
