//---------------------------------------------
// instSimulator.cpp
//---------------------------------------------

#include "instSimulator.h"
#include "boatSimulator.h"
#include <myDebug.h>

instSimulator instruments;
	// global instance

depthInst		i_depth;
logInst			i_log;
windInst		i_wind;
compassInst		i_compass;
gpsInst			i_gps;
autopilotInst	i_autopilot;
engineInst		i_engine;
gensetInst		i_genset;


void instSimulator::init(tNMEA2000 *nmea2000)
{
	display(0,"instSimulator::init() started",0);
	proc_entry();

	m_nmea2000 = nmea2000;
	boat.init();

	m_inst[INST_DEPTH]		= &i_depth;
	m_inst[INST_LOG]		= &i_log;
	m_inst[INST_WIND]		= &i_wind;
	m_inst[INST_COMPASS]	= &i_compass;
	m_inst[INST_GPS]		= &i_gps;
	m_inst[INST_AUTOPILOT]	= &i_autopilot;
	m_inst[INST_ENGINE]		= &i_engine;
	m_inst[INST_GENSET]		= &i_genset;

	proc_leave();
	display(0,"instSimulator::init() finished",0);
}


void instSimulator::run()
{
	boat.run();
	if (boat.running())
	{
		for (int i=0; i<NUM_INSTRUMENTS; i++)
		{
			delay(10);
			instBase *inst = m_inst[i];
			if (0 && inst->doProtocol(PROTOCOL_2000))
				inst->send2000(m_nmea2000);
			if (inst->doProtocol(PROTOCOL_0183))
				inst->send0183();
		}
	}
}


// end of instSimulator.cpp
