//---------------------------------------------
// aisTargets.cpp
//---------------------------------------------
// Implementation of the virtual AIS target simulator.
// See aisTargets.h for the overall design.

#include "aisTargets.h"
#include "boatSimulator.h"
#include "instSimulator.h"
#include "boatBinary.h"
#include <myDebug.h>
#include <math.h>


aisSimulator ais_targets;


//----------------------------------------------------
// tuning constants
//----------------------------------------------------

#define AIS_DEFAULT_NUM			5
#define AIS_DEFAULT_RATE		3.0		// seconds between bursts (see AIS_RATE)
#define AIS_DEFAULT_MIN_CPA		0.5		// NM standoff for normal vboats
#define AIS_DEFAULT_RANGE		4.0		// NM outer range (see AIS_RANGE); vboats recycle beyond it

#define AIS_MIN_ANNULUS			1.0		// range is kept at least this far above min_cpa
#define AIS_SPAWN_MIN_FRAC		0.5		// spawn between these fractions of the current range
#define AIS_SPAWN_MAX_FRAC		0.9

#define AIS_TURN_RATE_SLOW		8.0		// degrees per slice when gently steering
#define AIS_TURN_RATE_FAST		20.0	// degrees per slice inside the guard ring
#define AIS_COLLIDE_TURN_RATE	30.0	// degrees per slice for a collision pursuer
#define AIS_COLLIDE_MIN_SPEED	8.0		// knots minimum closing speed in COLLIDE mode


//----------------------------------------------------
// vessel dictionary (persistent recurring vboats)
//----------------------------------------------------
// Names are pre-folded to the AIS 6-bit charset (uppercase, <= 20 chars).
// MMSIs use the Panama MID (351) with a clearly-fake 9xxxx block so they
// never collide with real traffic.  Ship types are AIS vessel-type codes:
//   30=fishing 36=sailing 37=pleasure 52=tug 60=passenger 70=cargo 90=other

typedef struct
{
	const char *name;
	const char *callsign;
	uint32_t	mmsi;
	uint8_t		ship_type;
	uint8_t		length_m;
	uint8_t		beam_m;
} aisVessel;


static const aisVessel AIS_DICT[] =
{
	{ "SHIP HAPPENS",		"HP9001",	351900001,	37,	12,	4 },
	{ "COD FATHER",			"HP9002",	351900002,	30,	18,	6 },
	{ "PIER PRESSURE",		"HP9003",	351900003,	37,	14,	4 },
	{ "NAUTI BUOY",			"HP9004",	351900004,	36,	11,	4 },
	{ "KNOT ON CALL",		"HP9005",	351900005,	37,	13,	4 },
	{ "SEAS THE DAY",		"HP9006",	351900006,	36,	10,	3 },
	{ "AQUA HOLIC",			"HP9007",	351900007,	37,	15,	5 },
	{ "REEL THERAPY",		"HP9008",	351900008,	30,	16,	5 },
	{ "FISHFUL THINKING",	"HP9009",	351900009,	30,	20,	6 },
	{ "BREAKING WIND",		"HP9010",	351900010,	36,	12,	4 },
	{ "MISS BEHAVIN",		"HP9011",	351900011,	37,	14,	5 },
	{ "WATER YOU DOING",	"HP9012",	351900012,	37,	13,	4 },
	{ "SANDY BOTTOM",		"HP9013",	351900013,	36,	11,	4 },
	{ "SALTY DOG",			"HP9014",	351900014,	30,	17,	5 },
	{ "MOORE FUN",			"HP9015",	351900015,	37,	15,	5 },
	{ "BOW MOVEMENT",		"HP9016",	351900016,	37,	12,	4 },
	{ "VITAMIN SEA",		"HP9017",	351900017,	36,	10,	3 },
	{ "LIQUID ASSET",		"HP9018",	351900018,	60,	24,	7 },
	{ "WIND SEEKER",		"HP9019",	351900019,	36,	13,	4 },
	{ "ISLAND TIME",		"HP9020",	351900020,	60,	22,	7 },
	{ "BLACK PEARL",		"HP9021",	351900021,	70,	28,	8 },
	{ "SERENITY NOW",		"HP9022",	351900022,	37,	14,	4 },
	{ "PLAN SEA",			"HP9023",	351900023,	36,	11,	4 },
	{ "HAPPY HOOKER",		"HP9024",	351900024,	30,	18,	6 },
	{ "TERN LEFT",			"HP9025",	351900025,	52,	20,	7 },
};

#define AIS_DICT_SIZE	((int)(sizeof(AIS_DICT) / sizeof(AIS_DICT[0])))


//----------------------------------------------------
// small helpers
//----------------------------------------------------

static inline double deg2rad(double d)	{ return d * M_PI / 180.0; }
static inline double rad2deg(double r)	{ return r * 180.0 / M_PI; }


static float normDeg(float d)
{
	while (d < 0.0)		d += 360.0;
	while (d >= 360.0)	d -= 360.0;
	return d;
}


static float turnToward(float cur, float desired, float max_rate)
	// turn cur toward desired by at most max_rate degrees, shortest way around
{
	float diff = desired - cur;
	while (diff > 180.0)	diff -= 360.0;
	while (diff < -180.0)	diff += 360.0;
	if (diff > max_rate)	diff = max_rate;
	if (diff < -max_rate)	diff = -max_rate;
	return normDeg(cur + diff);
}


//----------------------------------------------------
// geometry relative to the sboat
//----------------------------------------------------

float aisSimulator::rangeToBoatNM(int i)
{
	aisTarget *t = &m_targets[i];
	waypoint_t boat_wp = { "boat", (float) boat_sim.getLat(), (float) boat_sim.getLon() };
	return boat_sim.distanceTo(t->lat, t->lon, &boat_wp);
}


float aisSimulator::bearingBoatToTargetDeg(int i)
	// true bearing FROM the sboat TO the vboat (i.e. the direction "away from boat")
{
	aisTarget *t = &m_targets[i];
	waypoint_t tgt_wp = { "tgt", (float) t->lat, (float) t->lon };
	return boat_sim.headingTo(boat_sim.getLat(), boat_sim.getLon(), &tgt_wp);
}


//----------------------------------------------------
// lifecycle
//----------------------------------------------------

void aisSimulator::init()
{
	m_enabled = false;
	m_num_targets = AIS_DEFAULT_NUM;
	m_rate_secs = AIS_DEFAULT_RATE;
	m_min_cpa = AIS_DEFAULT_MIN_CPA;
	m_range = AIS_DEFAULT_RANGE;
	m_collide_request = false;
	m_next_dict = 0;
	m_last_update_ms = 0;
	m_last_tx_ms = 0;
	m_tx_interval_ms = (uint32_t)(m_rate_secs * 1000.0);
	m_sched_len = 0;
	m_sched_pos = 0;
	clearTargets();
}


void aisSimulator::clearTargets()
{
	for (int i = 0; i < MAX_AIS_TARGETS; i++)
	{
		m_targets[i].active = false;
		m_targets[i].collide = false;
		m_targets[i].sched_msg = AIS_MSG_NONE;
	}
	m_sched_len = 0;
	m_sched_pos = 0;
}


void aisSimulator::spawnTarget(int i)
	// place vboat slot i at a random bearing and range from the sboat,
	// drawing the next un-used vessel from the dictionary
{
	aisTarget *t = &m_targets[i];

	// find the next dictionary vessel not currently used by another slot

	int tries = 0;
	uint8_t di = m_next_dict;
	while (tries < AIS_DICT_SIZE)
	{
		bool used = false;
		for (int j = 0; j < MAX_AIS_TARGETS; j++)
		{
			if (j != i && m_targets[j].active && m_targets[j].dict_index == di)
			{
				used = true;
				break;
			}
		}
		if (!used)
			break;
		di = (di + 1) % AIS_DICT_SIZE;
		tries++;
	}
	m_next_dict = (di + 1) % AIS_DICT_SIZE;

	const aisVessel *v = &AIS_DICT[di];

	t->active = true;
	t->dict_index = di;
	t->mmsi = v->mmsi;
	t->name = v->name;
	t->callsign = v->callsign;
	t->ship_type = v->ship_type;
	t->length_m = v->length_m;
	t->beam_m = v->beam_m;
	t->collide = false;
	t->sched_msg = AIS_MSG_NONE;

	// random position: bearing 0..359, range within a fraction of the outer range

	float bearing = (float) random(360);
	float min_r = m_range * AIS_SPAWN_MIN_FRAC;
	float max_r = m_range * AIS_SPAWN_MAX_FRAC;
	int span10 = (int)((max_r - min_r) * 10.0);
	float range = min_r + (span10 > 0 ? (float) random(0, span10) / 10.0 : 0.0);

	double blat = boat_sim.getLat();
	double blon = boat_sim.getLon();
	double br = deg2rad(bearing);
	t->lat = blat + (range / 60.0) * cos(br);
	t->lon = blon + (range / 60.0) * sin(br) / cos(deg2rad(blat));

	// random course and speed

	t->cog = (float) random(360);
	t->sog = 2.0 + (float) random(0, 100) / 10.0;	// 2.0 .. 11.9 knots
	t->heading = t->cog;

	display(0,"AIS spawn slot(%d) %s mmsi(%d) rng(%0.1f) brg(%0.0f) cog(%0.0f) sog(%0.1f)",
		i, t->name, t->mmsi, range, bearing, t->cog, t->sog);
}


//----------------------------------------------------
// motion
//----------------------------------------------------

void aisSimulator::deadReckon(int i, double elapsed_secs)
{
	aisTarget *t = &m_targets[i];
	if (t->sog == 0.0)
		return;

	const double EARTH_RADIUS = 6371000.0;	// meters
	const double KNOTS_TO_MPS = 0.514444;

	double distance_m = t->sog * KNOTS_TO_MPS * elapsed_secs;
	double cog_rad = deg2rad(t->cog);
	double delta_lat = (distance_m * cos(cog_rad)) / EARTH_RADIUS;
	double delta_lon = (distance_m * sin(cog_rad)) / (EARTH_RADIUS * cos(deg2rad(t->lat)));
	t->lat += rad2deg(delta_lat);
	t->lon += rad2deg(delta_lon);
	t->heading = t->cog;
}


void aisSimulator::steer(int i, double elapsed_secs)
{
	aisTarget *t = &m_targets[i];
	float range = rangeToBoatNM(i);

	if (t->collide)
	{
		// pure-pursuit toward the sboat's CURRENT position: recomputed every
		// slice this holds a near-constant bearing with decreasing range,
		// which is exactly what a receiver CPA alarm is built to detect.

		float to_boat = normDeg(bearingBoatToTargetDeg(i) + 180.0);
		t->cog = turnToward(t->cog, to_boat, AIS_COLLIDE_TURN_RATE);
		if (t->sog < AIS_COLLIDE_MIN_SPEED)
			t->sog = AIS_COLLIDE_MIN_SPEED;
		t->heading = t->cog;
		return;
	}

	// normal: a soft repulsion that keeps the vboat outside the guard ring so
	// it never trips the receiver's CPA alarm (set AIS_MIN_CPA above that ring).

	if (range < m_min_cpa * 3.0)
	{
		float away = bearingBoatToTargetDeg(i);		// direction pointing away from the sboat
		float rate = (range < m_min_cpa * 1.5) ? AIS_TURN_RATE_FAST : AIS_TURN_RATE_SLOW;
		t->cog = turnToward(t->cog, away, rate);
	}
	t->heading = t->cog;
}


void aisSimulator::ensurePopulation()
{
	bool changed = false;
	for (int i = 0; i < MAX_AIS_TARGETS; i++)
	{
		bool should = (i < m_num_targets);
		if (should && !m_targets[i].active)
		{
			spawnTarget(i);
			changed = true;
		}
		else if (!should && m_targets[i].active)
		{
			m_targets[i].active = false;
			m_targets[i].collide = false;
			changed = true;
		}
	}
	if (changed)
		m_sched_pos = m_sched_len;		// force a reshuffle next scheduler tick
}


//----------------------------------------------------
// shuffled round-robin scheduler
//----------------------------------------------------

void aisSimulator::buildSchedule()
	// one entry per active (target x message-type) pair, then shuffled.
	// this guarantees every vboat's full picture is seen within
	// num_active * 3 bursts while still looking random.
{
	m_sched_len = 0;
	for (int i = 0; i < MAX_AIS_TARGETS; i++)
	{
		if (!m_targets[i].active)
			continue;
		m_sched[m_sched_len++] = (uint8_t)(i * 4 + AIS_MSG_POSITION);
		m_sched[m_sched_len++] = (uint8_t)(i * 4 + AIS_MSG_STATIC_A);
		m_sched[m_sched_len++] = (uint8_t)(i * 4 + AIS_MSG_STATIC_B);
	}

	// Fisher-Yates shuffle

	for (int i = m_sched_len - 1; i > 0; i--)
	{
		int j = random(i + 1);
		uint8_t tmp = m_sched[i];
		m_sched[i] = m_sched[j];
		m_sched[j] = tmp;
	}
	m_sched_pos = 0;
}


uint32_t aisSimulator::nextTxIntervalMs()
{
	uint32_t base = (uint32_t)(m_rate_secs * 1000.0);
	if (base < 1000)
		base = 1000;
	uint32_t jitter = random(0, (long)(base / 2) + 1);
	return base + jitter;
}


void aisSimulator::runScheduler()
{
	uint32_t now = millis();
	if (now - m_last_tx_ms < m_tx_interval_ms)
		return;

	if (m_sched_pos >= m_sched_len)
		buildSchedule();
	if (m_sched_len == 0)
		return;

	uint8_t entry = m_sched[m_sched_pos++];
	int ti = entry / 4;
	int msg = entry % 4;
	if (m_targets[ti].active)
		m_targets[ti].sched_msg = (uint8_t) msg;

	m_last_tx_ms = now;
	m_tx_interval_ms = nextTxIntervalMs();
}


//----------------------------------------------------
// tick
//----------------------------------------------------

void aisSimulator::update()
{
	uint32_t now = millis();
	double elapsed_secs = (now - m_last_update_ms) / 1000.0;
	m_last_update_ms = now;
	if (elapsed_secs < 0.0 || elapsed_secs > 5.0)
		elapsed_secs = 1.0;		// clamp startup / stalls to one nominal slice

	if (!m_enabled)
		return;

	ensurePopulation();

	// apply a latched COLLIDE request to the first live vboat

	if (m_collide_request)
	{
		bool any = false;
		for (int i = 0; i < MAX_AIS_TARGETS; i++)
			if (m_targets[i].active && m_targets[i].collide)
				any = true;
		if (!any)
		{
			for (int i = 0; i < MAX_AIS_TARGETS; i++)
			{
				if (m_targets[i].active)
				{
					m_targets[i].collide = true;
					display(0,"AIS COLLIDE engaged on %s mmsi(%d)",
						m_targets[i].name, m_targets[i].mmsi);
					break;
				}
			}
		}
	}

	// move every vboat, recycle any that leave the outer range,
	// then steer (standoff or pursuit)

	for (int i = 0; i < MAX_AIS_TARGETS; i++)
	{
		if (!m_targets[i].active)
			continue;

		deadReckon(i, elapsed_secs);

		if (rangeToBoatNM(i) > m_range)
		{
			bool was_collide = m_targets[i].collide;
			spawnTarget(i);
			m_targets[i].collide = was_collide;	// keep testing until COLLIDE=0
			m_sched_pos = m_sched_len;			// reshuffle (mmsi changed)
		}

		steer(i, elapsed_secs);
	}

	// clear last slice's transmit flags, then let the scheduler mark
	// (at most) one vboat to transmit one message this slice

	for (int i = 0; i < MAX_AIS_TARGETS; i++)
		m_targets[i].sched_msg = AIS_MSG_NONE;

	runScheduler();
}


//----------------------------------------------------
// aggregated binary packet for the wxPerl AIS window
//----------------------------------------------------

void aisSimulator::sendBinaryAisState()
{
	if (!(g_BINARY & BINARY_TYPE_AIS))
		return;

	// count vboats that transmitted this slice

	int count = 0;
	for (int i = 0; i < MAX_AIS_TARGETS; i++)
		if (m_targets[i].active && m_targets[i].sched_msg != AIS_MSG_NONE)
			count++;
	if (count == 0)
		return;

	uint8_t buf[MAX_AIS_TARGETS * 80 + 16];
	int offset = startBinary(buf, BINARY_TYPE_AIS);

	offset = binaryUint8(buf, offset, (uint8_t) count);

	for (int i = 0; i < MAX_AIS_TARGETS; i++)
	{
		aisTarget *t = &m_targets[i];
		if (!t->active || t->sched_msg == AIS_MSG_NONE)
			continue;

		offset = binaryUint32	(buf, offset, t->mmsi);
		offset = binaryFixStr	(buf, offset, t->name, 20);
		offset = binaryDouble	(buf, offset, t->lat);
		offset = binaryDouble	(buf, offset, t->lon);
		offset = binaryFloat	(buf, offset, t->cog);
		offset = binaryFloat	(buf, offset, t->sog);
		offset = binaryFloat	(buf, offset, t->heading);
		offset = binaryFloat	(buf, offset, rangeToBoatNM(i));
		offset = binaryFloat	(buf, offset, bearingBoatToTargetDeg(i));
		offset = binaryUint8	(buf, offset, t->ship_type);
		offset = binaryUint8	(buf, offset, t->collide ? 1 : 0);
		offset = binaryUint8	(buf, offset, t->sched_msg);
	}

	endBinary(buf, offset);
	Serial.write(buf, offset);
	if (inst_sim.doTbEsp32())
		SERIAL_ESP32.write(buf, offset);
}


//----------------------------------------------------
// runtime setters
//----------------------------------------------------

void aisSimulator::setEnabled(bool on)
	// Called by the instSimulator layer as the AIS instrument's port mask
	// goes non-zero / zero.  Transition-guarded so redundant calls (a port
	// change that does not flip the on/off state) don't disturb the timing.
{
	if (on == m_enabled)
		return;
	m_enabled = on;
	if (on)
	{
		m_last_update_ms = millis();
		m_last_tx_ms = millis();
		m_tx_interval_ms = nextTxIntervalMs();
	}
	else
	{
		clearTargets();
		m_collide_request = false;
	}
	display(0,"AIS targets %s",on ? "ENABLED" : "disabled");
}


void aisSimulator::setNumTargets(int n)
{
	if (n < 0)					n = 0;
	if (n > MAX_AIS_TARGETS)	n = MAX_AIS_TARGETS;
	m_num_targets = n;
	m_sched_pos = m_sched_len;		// force reshuffle
	display(0,"AIS_N=%d",n);
}


void aisSimulator::setRate(float secs)
{
	if (secs < 1.0)
		secs = 1.0;
	m_rate_secs = secs;
	m_tx_interval_ms = nextTxIntervalMs();
	display(0,"AIS_RATE=%0.1f",secs);
}


void aisSimulator::setMinCPA(float nm)
{
	if (nm < 0.05)
		nm = 0.05;
	m_min_cpa = nm;
	if (m_range < m_min_cpa + AIS_MIN_ANNULUS)		// self-adjust range to keep a usable annulus
		m_range = m_min_cpa + AIS_MIN_ANNULUS;
	display(0,"AIS_MIN_CPA=%0.2f (range=%0.1f)",m_min_cpa,m_range);
}


void aisSimulator::setRange(float nm)
{
	float min_range = m_min_cpa + AIS_MIN_ANNULUS;	// must stay at least 1 NM beyond min_cpa
	if (nm < min_range)
		nm = min_range;
	m_range = nm;
	display(0,"AIS_RANGE=%0.1f",m_range);
}


void aisSimulator::setCollide(bool on)
{
	m_collide_request = on;
	if (!on)
	{
		for (int i = 0; i < MAX_AIS_TARGETS; i++)
			m_targets[i].collide = false;
	}
	display(0,"COLLIDE=%d",on);
}


void aisSimulator::dumpState()
{
	display(0,"AIS enabled(%d) targets(%d) rate(%0.1fs) min_cpa(%0.2fNM) range(%0.1fNM)",
		m_enabled, m_num_targets, m_rate_secs, m_min_cpa, m_range);
	for (int i = 0; i < MAX_AIS_TARGETS; i++)
	{
		aisTarget *t = &m_targets[i];
		if (!t->active)
			continue;
		display(0,"  [%d] %-18s mmsi(%d) rng(%0.2f) brg(%0.0f) cog(%0.0f) sog(%0.1f)%s",
			i, t->name, t->mmsi,
			rangeToBoatNM(i), bearingBoatToTargetDeg(i),
			t->cog, t->sog,
			t->collide ? " COLLIDE" : "");
	}
}


// end of aisTargets.cpp
