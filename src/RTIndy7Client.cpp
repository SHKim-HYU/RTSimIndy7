/*
 * RTIndy7Client.cpp
 *
 *  Created on: 2023. 06. 06.
 *      Author: Sunhong Kim
 */


#ifndef __XENO__
#define __XENO__
#endif
#include "RTIndy7Client.h"


JointInfo info;

// MR_Indy7 mr_indy7;

CS_Indy7 cs_sim_indy7;

// Xenomai RT tasks
RT_TASK indysim_task;
RT_TASK safety_task;
RT_TASK print_task;



//////////////////////////////////////////////////////////////////

void signal_handler(int signum);

void saveLogData(){}

int initAxes()
{
	for (int i = 0; i < NUM_AXIS; i++)
	{	
		Axis[i].setGearRatio(gearRatio[i]);
		Axis[i].setGearEfficiency(EFFICIENCY);
		Axis[i].setPulsePerRevolution(ENC_CORE);
		Axis[i].setTauADC(TauADC[i]);
		Axis[i].setTauK(TauK[i]);
		Axis[i].setZeroPos(zeroPos[i]);

		Axis[i].setDirQ(dirQ[i]);
		Axis[i].setDirTau(dirTau[i]);

		Axis[i].setConversionConstants();

		Axis[i].setTrajPeriod(period);
		
		Axis[i].setTarVelInCnt(0);
		Axis[i].setTarTorInCnt(0);
	}
	
	return 1;
}
/****************************************************************************/
void trajectory_generation(){
	/////////////Trajectory for Joint Space//////////////
    if(!Axis[0].trajInitialized())
    {
	    switch(motion)
	    {
	    case 1:
	    	info.q_target(0)=1.5709; 	info.q_target(1)=-0.4071; 	info.q_target(2)=0.4071;
	    	info.q_target(3)=1.5709; 	info.q_target(4)=1.5709; 	info.q_target(5)=1.5709;
	    	traj_time = 3.0;
	    	motion++;
	        break;
	    case 2:
	    	info.q_target(0)=0.0; 	info.q_target(1)=0.0; 	info.q_target(2)=0.0;
	    	info.q_target(3)=0.0; 	info.q_target(4)=0.0; 	info.q_target(5)=0.0;

	    	// info.q_target(0)=-1.5709; 	info.q_target(1)=0.4071; 	info.q_target(2)=-0.4071;
	    	// info.q_target(3)=-1.5709; 	info.q_target(4)=-1.5709; 	info.q_target(5)=-1.5709;
	    	traj_time = 3.0;
	    	motion++;
	    	// motion=1;
	        break;
	    case 3:
	    	info.q_target(0)=-1.5709; 	info.q_target(1)=0.4071; 	info.q_target(2)=-0.4071;
	    	info.q_target(3)=-1.5709; 	info.q_target(4)=-1.5709; 	info.q_target(5)=-1.5709;

	    	// info.q_target(0)=1.5709; 	info.q_target(1)=-0.4071; 	info.q_target(2)=0.4071;
	    	// info.q_target(3)=1.5709; 	info.q_target(4)=1.5709; 	info.q_target(5)=1.5709;
	    	traj_time = 3.0;
	    	motion++;
	        break;
	    case 4:
	    	info.q_target(0)=0.0; 	info.q_target(1)=0.0; 	info.q_target(2)=0.0;
	    	info.q_target(3)=0.0; 	info.q_target(4)=0.0; 	info.q_target(5)=0.0;

	    	traj_time = 3.0;
	    	motion=1;
	    	break;
	    default:
	    	info.q_target(0)=info.act.q(0); 	info.q_target(1)=info.act.q(1); 	info.q_target(2)=info.act.q(2);
	    	info.q_target(3)=info.act.q(3); 	info.q_target(4)=info.act.q(4); 	info.q_target(5)=info.act.q(5);
	    	motion=1;
	    	break;
	    }
	}

	for(int i=0;i<NUM_AXIS;i++)
	{
		if(!Axis[i].trajInitialized())
		{
			Axis[i].setTrajInitialQuintic();
			Axis[i].setTarPosInRad(info.q_target(i));
			Axis[i].setTarVelInRad(0);
			Axis[i].setTrajTargetQuintic(traj_time);
		}

		Axis[i].TrajQuintic();

		info.des.q(i)=Axis[i].getDesPosInRad();
		info.des.q_dot(i)=Axis[i].getDesVelInRad();
		info.des.q_ddot(i)=Axis[i].getDesAccInRad();
	}
}


int compute()
{


	return 0;
}


// IndySim task
void indysim_run(void *arg)
{
	RTIME now, previous=0;
	RTIME beginCycle, endCycle;
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	info.des.q = JVec::Zero();
	info.des.q_dot = JVec::Zero();
	info.des.q_ddot = JVec::Zero();
	info.des.F = Vector6d::Zero();
	info.des.F_CB = Vector6d::Zero();
	
	info.act.q = JVec::Zero();
	info.act.q_dot = JVec::Zero();
	info.act.tau = JVec::Zero();

	// Load the shared library
    void* fd_handle = dlopen("../lib/URDF2CASADI/indy7/indy7_fd.so", RTLD_LAZY);
    if (fd_handle == 0) {
        throw std::runtime_error("Cannot open indy7_fd.so");
    }
    
    // Reset error
    dlerror();

    // Function evaluation
    eval_t fd_eval = (eval_t)dlsym(fd_handle, "aba");
    if (dlerror()) {
        throw std::runtime_error("Function evaluation failed.");
    }
    
    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = NUM_AXIS;
    casadi_int sz_res = NUM_AXIS;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* fd_arg[3*sz_arg];

    double* fd_res[sz_res];
    casadi_int fd_iw[sz_iw];
    double fd_w[sz_w];
    
    int fd_mem = 0;  // No thread-local memory management    

    double temp_q[NUM_AXIS], temp_q_dot[NUM_AXIS], temp_tau[NUM_AXIS];
    
    JVec k1, k2, k3, k4;
    Jacobian J_b;
    MassMat M, C;
    JVec G;

	JVec e = JVec::Zero();
    JVec edot = JVec::Zero();
    JVec eint = JVec::Zero();

    // Set output buffers
    double fd_values[NUM_AXIS];
    for (casadi_int i = 0; i < NUM_AXIS; ++i) {
        fd_res[i] = &fd_values[i];
    }

	int cnt = 0;    

    // // Free the handle
    // dlclose(handle);

	Hinf_Kp = JMat::Zero();
    Hinf_Kv = JMat::Zero();
    Hinf_K_gamma = JMat::Zero();

    for (int i=0; i<NUM_AXIS; ++i)
    {
        switch(i)
        {
        case 0:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_1 ;
            break;
        case 1:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_2 ;

            break;
        case 2:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_3 ;

            break;
        case 3:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_4 ;

            break;
        case 4:
              Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_5 ;

            break;
        case 5:
            Hinf_Kp(i,i) = 100.0;
            Hinf_Kv(i,i) = 20.0;
            Hinf_K_gamma(i,i) = 1.0+1.0/invL2sqr_6 ;

            break;
        }
    }

	// loop
	while(1)
	{
			beginCycle = rt_timer_read();

			/////////////////  RK4   //////////////////
			if(cnt == 0)
			{
				// state update
				JVec _q = info.act.q;
				// JVec _q_dot = info.act.q_dot;
				JVec _q_dot = JVec::Zero();
				JVec _tau = JVec::Zero();

				// 1st stage
			    k1 = cs_sim_indy7.computeFD(_q, _q_dot, _tau);
		    	JVec _q1 = info.act.q + 0.5 * period * info.act.q_dot;		// period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
		    	JVec _q_dot1 = info.act.q_dot + 0.5 * period * k1; // k2(q_dot)
		    	
			    // 2nd stage
			    k2 = cs_sim_indy7.computeFD(_q1, _q_dot1, _tau);
				JVec _q2 = info.act.q + 0.5 * period * _q_dot1;
		    	JVec _q_dot2 = info.act.q_dot + 0.5 * period * k2;
		    	
		    	// 3th stage
			    k3 = cs_sim_indy7.computeFD(_q2, _q_dot2, _tau);
				JVec _q3 = info.act.q + period * _q_dot2;
		    	JVec _q_dot3 = info.act.q_dot + period * k3;
		    	
			   	// 4th stage
			    k4 = cs_sim_indy7.computeFD(_q3, _q_dot3, _tau);
		    	info.nom.q = info.act.q + (period / 6.0) * (info.act.q_dot + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
		    	info.nom.q_dot = info.act.q_dot + (period / 6.0) * (k1 + 2 * (k2 + k3) + k4);
				
		    	// update robot & compute dynamics
		    	cs_sim_indy7.updateRobot(info.nom.q, info.nom.q_dot);
		    	
		    	M = cs_sim_indy7.getM();
		    	C = cs_sim_indy7.getC();
		    	J_b = cs_sim_indy7.getJ_b();
		    	G = cs_sim_indy7.getG();

				for(int i=0; i<NUM_AXIS;i++)
				{
					Axis[i].setCurrentPosInCnt(info.nom.q(i));
					Axis[i].setCurrentVelInCnt(info.nom.q_dot(i));
					Axis[i].setCurrentTorInCnt(info.nom.tau(i));
					
					Axis[i].setCurrentTime(gt);
				}

				// Trajectory Generation
				trajectory_generation();
				
				//[ToDo] Add MPC Function 
				compute();	

				// Calculate Joint controller
				e = info.des.q-info.nom.q;
				edot = info.des.q_dot-info.nom.q_dot;
				eint = eint + e*period;

				info.nom.tau_ext = J_b.transpose()*info.act.F;

			    JVec ddq_ref = info.des.q_ddot + Hinf_Kv*edot + Hinf_Kp*e;
			    JVec dq_ref = info.des.q_dot + Hinf_Kv*edot + Hinf_Kp*e;

			    info.nom.tau = M*ddq_ref + (Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint) - info.nom.tau_ext;
			    // info.nom.tau = M*ddq_ref + (Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
			    // info.nom.tau = M*ddq_ref + C*info.nom.q_dot;

				cnt++;
			}
			else if(cnt==1)
			{
				// 1st stage
			    k1 = cs_sim_indy7.computeFD(info.nom.q, info.nom.q_dot, info.nom.tau);
		    	JVec _q1 = info.nom.q + 0.5 * period * info.nom.q_dot;
		    	JVec _q_dot1 = info.nom.q_dot + 0.5 * period * k1;

			    // 2nd stage
			    k2 = cs_sim_indy7.computeFD(_q1, _q_dot1, info.nom.tau);
		    	JVec _q2 = info.nom.q + 0.5 * period * _q_dot1;
		    	JVec _q_dot2 = info.nom.q_dot + 0.5 * period * k2;

		    	// 3th stage
		    	k3 = cs_sim_indy7.computeFD(_q2, _q_dot2, info.nom.tau);
		    	JVec _q3 = info.nom.q + period * _q_dot2;
		    	JVec _q_dot3 = info.nom.q_dot + period * k3;

			   	// 4th stage
			    k4 = cs_sim_indy7.computeFD(_q3, _q_dot3, info.nom.tau);
		    	info.nom.q = info.nom.q + (period / 6.0) * (info.nom.q_dot + 2 * (_q_dot1 + _q_dot2) + _q_dot3);
		    	info.nom.q_dot = info.nom.q_dot + (period / 6.0) * (k1 + 2 * (k2 + k3) + k4);

		    	// update robot & compute dynamics
		    	cs_sim_indy7.updateRobot(info.nom.q, info.nom.q_dot);
		    	
		    	M = cs_sim_indy7.getM();
		    	C = cs_sim_indy7.getC();
		    	J_b = cs_sim_indy7.getJ_b();
		    	G = cs_sim_indy7.getG();

				for(int i=0; i<NUM_AXIS;i++)
				{
					Axis[i].setCurrentPosInCnt(info.nom.q(i));
					Axis[i].setCurrentVelInCnt(info.nom.q_dot(i));
					Axis[i].setCurrentTorInCnt(info.nom.tau(i));
					
					Axis[i].setCurrentTime(gt);
				}

				// Trajectory Generation
				trajectory_generation();
				
				//[ToDo] Add MPC Function 
				compute();
		    	
			    // Calculate Joint controller
				e = info.des.q-info.nom.q;
				edot = info.des.q_dot-info.nom.q_dot;
				eint = eint + e*period;

				info.nom.tau_ext = J_b.transpose()*info.act.F;

			    JVec ddq_ref = info.des.q_ddot + Hinf_Kv*edot + Hinf_Kp*e;
			    JVec dq_ref = info.des.q_dot + Hinf_Kv*edot + Hinf_Kp*e;
			    // rt_printf("ddq_ref: %lf, %lf, %lf, %lf, %lf, %lf \n", ddq_ref(0), ddq_ref(1), ddq_ref(2), ddq_ref(3), ddq_ref(4), ddq_ref(5));

			    info.nom.tau = M*ddq_ref + (Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint) - info.nom.tau_ext;
			    // info.nom.tau = M*ddq_ref + (Hinf_K_gamma)*(edot + Hinf_Kv*e + Hinf_Kp*eint);
			    // info.nom.tau = M*ddq_ref + C*info.nom.q_dot;
			}

			endCycle = rt_timer_read();
			periodIndysim = (unsigned long) endCycle - beginCycle;
			
			gt+= period;
			rt_task_wait_period(NULL); //wait for next cycle
	}
		
}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState[NUM_AXIS]={0,};
	
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*400);
	
	string filename = "robot_log.csv";
	ifstream checkFile(filename);

	if (checkFile.is_open())
	{
		checkFile.close();
		remove(filename.c_str());
	}

	ofstream newFile(filename);
	if(newFile.is_open())
	{
		newFile<<"Time, q_r1, q_r2, q_r3, q_r4, q_r5, q_r6, q_r3, dq_r1, dq_r2, dq_r3, dq_r4, dq_r5, dq_r6, t_r1, t_r2, t_r3, t_r4, t_r5, t_r6, G_r1, G_r2, G_r3, G_r4, G_r5, G_r6, "
		"q_n1, q_n2, q_n3, q_n4, q_n5, q_n6, q_n3, dq_n1, dq_n2, dq_n3, dq_n4, dq_n5, dq_n6, t_n1, t_n2, t_n3, t_n4, t_n5, t_n6, "
		"qd1, qd2, qd3, qd4, qd5, qd6, qd3, dqd1, dqd2, dqd3, dqd4, dqd5, dqd6\n";
		newFile.close();
	}


	ofstream csvFile(filename, ios_base::app);


	while (1)
	{
		rt_task_wait_period(NULL); //wait for next cycle

		now = rt_timer_read();
		step=(unsigned long)(now - previous) / 1000000;
		itime+=step;
		previous=now;

		rt_printf("Time=%0.3lfs, cycle_dt=%lius,  overrun=%d\n", gt, periodCycle/1000, overruns);
		rt_printf("IndySim_dt=%lius\n",periodIndysim/1000);

		for(int j=0; j<NUM_AXIS; ++j){
			rt_printf("ID: %d", j);
			rt_printf("\t NomPos: %lf, NomVel: %lf \n",info.nom.q(j), info.nom.q_dot(j));
			rt_printf("\t DesPos: %lf, DesVel :%lf, DesAcc :%lf\n",info.des.q[j],info.des.q_dot[j],info.des.q_ddot[j]);
		// 	rt_printf("\t e: %lf, edot :%lf",info.des.q[j]-info.act.q[j],info.des.q_dot[j]-info.act.q_ddot[j]);
			// rt_printf("\t TarTor: %f, ",				TargetTorq[j]);
			rt_printf("\t TarTor: %f, ActTor: %lf, NomTor: %lf, ExtTor: %lf \n", info.des.tau(j), info.act.tau(j), info.nom.tau(j), info.act.tau_ext(j));
		}


		rt_printf("\n");

	}
}


/****************************************************************************/
void signal_handler(int signum)
{
	rt_task_delete(&print_task);

	rt_task_delete(&indysim_task);


	printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGWINCH)
		printf("╔═══════════════[SIGNAL INPUT SIGWINCH]══════════════╗\n");		
	else if(signum==SIGHUP)
		printf("╔════════════════[SIGNAL INPUT SIGHUP]═══════════════╗\n");
    printf("║                Servo drives Stopped!               ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	

    exit(1);
}


/****************************************************************************/
int main(int argc, char **argv)
{
	// Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_init(0, NULL);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGWINCH, signal_handler);
	signal(SIGHUP, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	// cycle_ns = 1000000; // nanosecond -> 1kHz
	cycle_ns = 250000; // nanosecond -> 4kHz
	// cycle_ns = 125000; // nanosecond -> 8kHz
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

	cs_sim_indy7=CS_Indy7();
	cs_sim_indy7.CSSetup("../lib/URDF2CASADI/indy7/indy7.json");
	

	MAX_TORQUES<<MAX_TORQUE_1,MAX_TORQUE_2,MAX_TORQUE_3,MAX_TORQUE_4,MAX_TORQUE_5,MAX_TORQUE_6;

	// For trajectory interpolation
	initAxes();

	// RTIndy7_task: create and start
	printf("Now running rt task ...\n");

	// RTIndy7 simulation
	rt_task_create(&indysim_task, "indysim_task", 0, 99, 0);
	rt_task_start(&indysim_task, &indysim_run, NULL);


	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 70, 0);
	rt_task_start(&print_task, &print_run, NULL);
	

	// Must pause here
	pause();
	/*
	while (1)
	{
		usleep(1e5);
	}
	*/
	// Finalize
	signal_handler(0);

    return 0;
}



