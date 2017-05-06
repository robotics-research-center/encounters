
#ifndef THREADCOMM_H_
#define THREADCOMM_H_

class comm
{
	public:
	bool loop_wait = true;
	bool viso_wait_flag = true;
	bool loop_write_done = false;
	bool g2o_loop_flag = false;
	bool can_start_viso=false;
        
        bool in_dloop_g2oedge=false;
        bool in_viso_g2oedge=false;

	bool first_loop_just_found=false;
	bool inserting_into_isam=false;
	bool normal_postloop_flow=false;
        int Nfirstloop=0;
        int Nsecondloop=0;
        
};
#endif
