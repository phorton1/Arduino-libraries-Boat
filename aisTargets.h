//---------------------------------------------
// aisTargets.h
//---------------------------------------------
// Virtual "other boat" AIS target simulator.
//
// Owns a small set of persistent virtual vessels (vboats) that
// circle in and out of range of the simulated boat (sboat), and
// drives the AIS output implemented in the aisInst::send0183() and
// aisInst::send2000() methods.
//
// The model is protocol independent. It is ticked once per simulator
// time slice by instSimulator::run(), but ONLY while enabled. There is no
// user "AIS on/off" command: the (upper) instSimulator layer calls
// setEnabled() as the AIS virtual instrument's port mask goes non-zero /
// zero, so the vboats run exactly when the AIS instrument is turned on --
// on ANY port, SeaTalk included (even though SeaTalk transmits no AIS).
// This keeps the model in the boatSimulator layer, ignorant of
// instSimulator, and the enable uniform/polymorphic across all ports.
//
// Each time slice the scheduler may mark ONE vboat to transmit ONE
// message type (position, static-A, or static-B) this slice, using a
// shuffled round-robin over all (vboat x message-type) pairs. Both the
// NMEA0183 and NMEA2000 senders read the same per-slice flag so a given
// burst is consistent across protocols. A separate aggregated binary
// packet (BINARY_TYPE_AIS) is emitted on the same trigger for the
// wxPerl AIS window.

#pragma once
#include <Arduino.h>


#define MAX_AIS_TARGETS			8		// compile time array cap

// message types a vboat can transmit in a single burst

#define AIS_MSG_NONE			0
#define AIS_MSG_POSITION		1		// AIS msg 18 / PGN 129039
#define AIS_MSG_STATIC_A		2		// AIS msg 24 part A / PGN 129809 (name)
#define AIS_MSG_STATIC_B		3		// AIS msg 24 part B / PGN 129810 (static)


typedef struct
{
	bool		active;
	uint8_t		dict_index;		// which AIS_DICT vessel occupies this slot
	uint32_t	mmsi;
	const char *name;			// -> AIS_DICT (already in AIS 6-bit charset)
	const char *callsign;
	uint8_t		ship_type;		// AIS vessel type code
	uint8_t		length_m;
	uint8_t		beam_m;
	double		lat;
	double		lon;
	float		cog;			// degrees true
	float		sog;			// knots
	float		heading;		// degrees true
	bool		collide;		// COLLIDE=1 override on this vboat
	uint8_t		sched_msg;		// AIS_MSG_* to transmit THIS slice (else AIS_MSG_NONE)
} aisTarget;


class aisSimulator
{
public:

	void init();

	void update();				// tick the model; call once per slice while AIS active
	void sendBinaryAisState();	// emit aggregated BINARY_TYPE_AIS packet (if enabled and any tx this slice)

	// runtime setters

	void setEnabled(bool on);			// driven by instSimulator when the AIS instrument port mask changes
	void setNumTargets(int n);			// AIS_N=n
	void setRate(float secs);			// AIS_RATE=secs   (>= 1.0)
	void setMinCPA(float nm);			// AIS_MIN_CPA=nm  (bumps range up to keep range >= min_cpa+1)
	void setRange(float nm);			// AIS_RANGE=nm    (clamped to >= min_cpa+1)
	void setCollide(bool on);			// COLLIDE=0/1

	// getters

	bool  getEnabled()		{ return m_enabled; }
	int   getNumTargets()	{ return m_num_targets; }
	float getRate()			{ return m_rate_secs; }
	float getMinCPA()		{ return m_min_cpa; }
	float getRange()		{ return m_range; }

	int   getMaxTargets()				{ return MAX_AIS_TARGETS; }
	aisTarget *getTarget(int i)			{ return &m_targets[i]; }

	void dumpState();		// AIS status report to the console

private:

	bool	 m_enabled;				// set by instSimulator when the AIS instrument port mask goes non-zero/zero
	int		 m_num_targets;
	float	 m_rate_secs;
	float	 m_min_cpa;
	float	 m_range;				// outer range (NM); vboats retire/recycle beyond it; kept >= m_min_cpa+1
	bool	 m_collide_request;		// COLLIDE=1 latched; applied to a live vboat in update()

	uint8_t	 m_next_dict;		// next dictionary vessel to hand out (cycles)
	uint32_t m_last_update_ms;	// for dead reckoning elapsed time
	uint32_t m_last_tx_ms;		// last burst time
	uint32_t m_tx_interval_ms;	// current (jittered) inter-burst interval

	aisTarget m_targets[MAX_AIS_TARGETS];

	// shuffled round-robin schedule over active (target x msgtype) pairs
	// each entry encodes target_index * 4 + msg_type

	uint8_t	 m_sched[MAX_AIS_TARGETS * 3];
	int		 m_sched_len;
	int		 m_sched_pos;

	void clearTargets();
	void spawnTarget(int i);
	void deadReckon(int i, double elapsed_secs);
	void steer(int i, double elapsed_secs);
	void ensurePopulation();
	void buildSchedule();
	void runScheduler();

	float rangeToBoatNM(int i);			// NM from vboat to sboat
	float bearingBoatToTargetDeg(int i);	// true bearing from sboat to vboat
	uint32_t nextTxIntervalMs();

};	// class aisSimulator


extern aisSimulator ais_targets;
	// static instance in aisTargets.cpp


// end of aisTargets.h
